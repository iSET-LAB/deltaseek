"""Render the extended abstract into an IEEE conference Word template.

The template (conference-template-a4.docx) is ISO/IEC 29500 *strict* OOXML, so
python-docx cannot be used and the document part is written directly.  The
template's own styles carry the IEEE formatting, including the automatic
numbering of headings, figures, tables and references, so the content below
supplies text only.

Usage:  python3 scripts/make_ieee_docx.py [template.docx] [out.docx]
                                         [--paper=a4|letter]

The paper size is set here rather than taken from the template, because a
US-letter template supplied as legacy binary .doc cannot be read at all.
Letter and A4 differ only in the page box; the IEEE margins and column
measures are identical.
"""

from __future__ import annotations

import re
import shutil
import struct
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FIGURES = ROOT / 'ieeeconf' / 'figures'

EMU_PER_PT = 12700

# --------------------------------------------------------------------------
# low-level XML helpers
# --------------------------------------------------------------------------


def esc(text: str) -> str:
    return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def run(text: str, *, b=False, i=False, sup=False, sub=False, font=None) -> str:
    props = ''
    if font:
        props += f'<w:rFonts w:ascii="{font}" w:hAnsi="{font}"/>'
    if b:
        props += '<w:b/>'
    if i:
        props += '<w:i/>'
    if sup:
        props += '<w:vertAlign w:val="superscript"/>'
    if sub:
        props += '<w:vertAlign w:val="subscript"/>'
    rpr = f'<w:rPr>{props}</w:rPr>' if props else ''
    return f'<w:r>{rpr}<w:t xml:space="preserve">{esc(text)}</w:t></w:r>'


# inline markup:  **bold**  *italic*  _{subscript}  ^{superscript}
_TOKEN = re.compile(r'(\*\*.+?\*\*|\*.+?\*|_\{.*?\}|\^\{.*?\})', re.S)


def runs(text: str) -> str:
    out = []
    for piece in _TOKEN.split(text):
        if not piece:
            continue
        if piece.startswith('**'):
            out.append(run(piece[2:-2], b=True))
        elif piece.startswith('*'):
            out.append(run(piece[1:-1], i=True))
        elif piece.startswith('_{'):
            out.append(run(piece[2:-1], sub=True))
        elif piece.startswith('^{'):
            out.append(run(piece[2:-1], sup=True))
        else:
            out.append(run(piece))
    return ''.join(out)


def para(style: str | None, body: str, sect: str = '') -> str:
    ppr = ''
    if style:
        ppr += f'<w:pStyle w:val="{style}"/>'
    ppr += sect
    ppr = f'<w:pPr>{ppr}</w:pPr>' if ppr else ''
    return f'<w:p>{ppr}{body}</w:p>'


def text_para(style: str | None, text: str, sect: str = '') -> str:
    return para(style, runs(text), sect)


# --------------------------------------------------------------------------
# OMML display equations
# --------------------------------------------------------------------------


def mr(text: str, upright: bool = False) -> str:
    rpr = '<m:rPr><m:sty m:val="p"/></m:rPr>' if upright else ''
    return f'<m:r>{rpr}<m:t xml:space="preserve">{esc(text)}</m:t></m:r>'


def m_sub(base: str, sub: str) -> str:
    return f'<m:sSub><m:sSubPr><m:ctrlPr/></m:sSubPr><m:e>{base}</m:e><m:sub>{sub}</m:sub></m:sSub>'


def m_sup(base: str, sup: str) -> str:
    return f'<m:sSup><m:sSupPr><m:ctrlPr/></m:sSupPr><m:e>{base}</m:e><m:sup>{sup}</m:sup></m:sSup>'


def m_subsup(base: str, sub: str, sup: str) -> str:
    return ('<m:sSubSup><m:sSubSupPr><m:ctrlPr/></m:sSubSupPr>'
            f'<m:e>{base}</m:e><m:sub>{sub}</m:sub><m:sup>{sup}</m:sup></m:sSubSup>')


def m_delim(items, beg='(', end=')', sep=',') -> str:
    inner = ''.join(f'<m:e>{i}</m:e>' for i in items)
    separator = f'<m:sepChr m:val="{esc(sep)}"/>' if sep else ''
    return ('<m:d><m:dPr>'
            f'<m:begChr m:val="{esc(beg)}"/>{separator}'
            f'<m:endChr m:val="{esc(end)}"/><m:ctrlPr/>'
            f'</m:dPr>{inner}</m:d>')


def m_limlow(base: str, lim: str) -> str:
    return ('<m:limLow><m:limLowPr><m:ctrlPr/></m:limLowPr>'
            f'<m:e>{base}</m:e><m:lim>{lim}</m:lim></m:limLow>')


def m_func(name: str, arg: str) -> str:
    return ('<m:func><m:funcPr><m:ctrlPr/></m:funcPr>'
            f'<m:fName>{name}</m:fName><m:e>{arg}</m:e></m:func>')


def equation(omml: str, number: str) -> str:
    """A numbered display equation using the template's `equation` tab stops."""
    tab = '<w:r><w:tab/></w:r>'
    num = ('<w:r><w:rPr><w:rFonts w:ascii="Times New Roman" '
           f'w:hAnsi="Times New Roman"/></w:rPr><w:t>({number})</w:t></w:r>')
    return para('equation', f'{tab}<m:oMath>{omml}</m:oMath>{tab}{num}')


Q, E, S, V, C, U, M, B = (mr(c) for c in 'QESVCUMB')

EQ1 = mr('q') + mr('=') + m_delim([m_sub(mr('q'), mr('b')), m_sub(mr('q'), mr('a'))])

EQ2 = (
    m_sup(mr('q'), mr('∗')) + mr('=')
    + m_func(
        m_limlow(mr('arg max', upright=True), mr('q') + mr('∈', upright=True) + Q),
        m_delim([
            U + m_delim([mr('q'), M + mr(', ') + m_sub(B, mr('t'))], sep='|')
            + mr(' − ') + mr('λ') + C + m_delim([mr('q')]),
        ], beg='[', end=']', sep=''),
    )
)

EQ3 = (
    V + m_delim([mr('e'), mr('q')]) + mr(' ≥ ') + m_sub(mr('τ'), mr('v'))
    + mr(', ') + mr('with', upright=True) + mr(' ')
    + m_sub(mr('τ'), mr('v')) + mr(' = 0.05')
)

EQ4 = (
    m_subsup(Q, mr('e'), mr('s')) + mr(' = ')
    + m_delim([
        mr('q') + mr(' ∈ ') + m_sub(Q, mr('s')) + mr(' : ')
        + V + m_delim([mr('e'), mr('q')]) + mr(' ≥ ') + m_sub(mr('τ'), mr('v')),
    ], beg='{', end='}', sep='')
)

EQ5 = (
    m_sub(C, mr('s')) + m_delim([mr('e')]) + mr(' = ')
    + m_func(
        m_limlow(mr('min', upright=True),
                 mr('q') + mr(' ∈ ') + m_subsup(Q, mr('e'), mr('s'))),
        m_sub(mr('d'), mr('nav', upright=True))
        + m_delim([m_sub(mr('q'), mr('0')), mr('q')]),
    )
)


# --------------------------------------------------------------------------
# images
# --------------------------------------------------------------------------


def png_size(path: Path) -> tuple[int, int]:
    head = path.read_bytes()[:33]
    return struct.unpack('>II', head[16:24])


class Media:
    def __init__(self) -> None:
        self.items: list[tuple[str, Path]] = []

    def add(self, path: Path) -> str:
        rid = f'rId{900 + len(self.items)}'
        self.items.append((rid, path))
        return rid


def picture(media: Media, path: Path, width_pt: float) -> str:
    rid = media.add(path)
    w, h = png_size(path)
    cx = int(width_pt * EMU_PER_PT)
    cy = int(cx * h / w)
    pid = 900 + len(media.items)
    return (
        '<w:r><w:drawing>'
        '<wp:inline distT="0" distB="0" distL="0" distR="0">'
        f'<wp:extent cx="{cx}" cy="{cy}"/>'
        '<wp:effectExtent l="0" t="0" r="0" b="0"/>'
        f'<wp:docPr id="{pid}" name="{path.stem}"/>'
        '<wp:cNvGraphicFramePr><a:graphicFrameLocks noChangeAspect="1"/></wp:cNvGraphicFramePr>'
        '<a:graphic><a:graphicData uri="http://purl.oclc.org/ooxml/drawingml/picture">'
        '<pic:pic>'
        f'<pic:nvPicPr><pic:cNvPr id="{pid}" name="{path.name}"/><pic:cNvPicPr/></pic:nvPicPr>'
        f'<pic:blipFill><a:blip r:embed="{rid}"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill>'
        f'<pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm>'
        '<a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr>'
        '</pic:pic></a:graphicData></a:graphic>'
        '</wp:inline></w:drawing></w:r>'
    )


def figure(media: Media, name: str, width_pt: float, caption: str, sect: str = '') -> str:
    img = para(None, picture(media, FIGURES / 'png' / f'{name}.png', width_pt),
               '<w:jc w:val="center"/>')
    return img + text_para('figurecaption', caption, sect)


# --------------------------------------------------------------------------
# section properties
# --------------------------------------------------------------------------

# A4 is the default because that is the template this was written against.
# The US-letter geometry differs only in the page box: the IEEE conference
# margins and column measures are the same on both.
PAPER_SIZES = {
    'a4': ('595.30pt', '841.90pt', '9'),
    'letter': ('612pt', '792pt', '1'),
}
PAPER = 'a4'

_PAGE = ('<w:pgSz w:w="{w}" w:h="{h}" w:code="{code}"/>'
         '<w:pgMar w:top="{top}" w:right="44.65pt" w:bottom="72pt" w:left="44.65pt"'
         ' w:header="36pt" w:footer="36pt" w:gutter="0pt"/>')


def _page(top):
    width, height, code = PAPER_SIZES[PAPER]
    return _PAGE.format(w=width, h=height, code=code, top=top)

def _sections():
    """Rebuild the section properties for the current paper size."""
    global SECT_TITLE, SECT_AUTHORS, SECT_2COL, SECT_1COL
    SECT_TITLE = ('<w:sectPr><w:footerReference w:type="first" r:id="rId8"/>'
              + _page('27pt')
                  + '<w:cols w:space="36pt"/><w:titlePg/><w:docGrid w:linePitch="360"/></w:sectPr>')
    SECT_AUTHORS = ('<w:sectPr><w:type w:val="continuous"/>' + _page('22.50pt')
                    + '<w:cols w:num="2" w:space="36pt"/><w:docGrid w:linePitch="360"/></w:sectPr>')
    SECT_2COL = ('<w:sectPr><w:type w:val="continuous"/>' + _page('54pt')
                 + '<w:cols w:num="2" w:space="18pt"/><w:docGrid w:linePitch="360"/></w:sectPr>')
    SECT_1COL = ('<w:sectPr><w:type w:val="continuous"/>' + _page('54pt')
                 + '<w:cols w:space="36pt"/><w:docGrid w:linePitch="360"/></w:sectPr>')


_sections()


# --------------------------------------------------------------------------
# the table
# --------------------------------------------------------------------------


def table_cell(text: str, width_pt: str, style: str, span: int = 1, bold=False) -> str:
    grid = f'<w:gridSpan w:val="{span}"/>' if span > 1 else ''
    body = runs(text) if not bold else run(text, b=True)
    return (f'<w:tc><w:tcPr><w:tcW w:w="{width_pt}" w:type="dxa"/>{grid}'
            '<w:vAlign w:val="center"/></w:tcPr>'
            + para(style, body, '<w:jc w:val="center"/>') + '</w:tc>')


def results_table() -> str:
    widths = ['54pt', '40pt', '40pt', '40pt']
    grid = [1080, 800, 800, 800]  # twips, as the template's own table does
    header = ['Class', 'chassis', 'wrist', 'both']
    rows = [
        ('trivial', '2/2', '2/2', '2/2'),
        ('height', '**0/2**', '2/2', '2/2'),
        ('incidence', '1/2', '2/2', '2/2'),
        ('occluded', '2/2', '2/2', '2/2'),
        ('all', '5/8', '8/8', '8/8'),
    ]
    trs = ['<w:tr><w:trPr><w:cantSplit/><w:tblHeader/><w:jc w:val="center"/></w:trPr>'
           + ''.join(table_cell(h, w, 'tablecolhead') for h, w in zip(header, widths))
           + '</w:tr>']
    for row in rows:
        trs.append('<w:tr><w:trPr><w:cantSplit/><w:jc w:val="center"/></w:trPr>'
                   + ''.join(table_cell(c, w, 'tablecopy') for c, w in zip(row, widths))
                   + '</w:tr>')
    borders = ''.join(
        f'<w:{side} w:val="single" w:sz="4" w:space="0" w:color="auto"/>'
        for side in ('top', 'bottom', 'insideH'))
    return (
        '<w:tbl><w:tblPr><w:tblW w:w="0pt" w:type="dxa"/><w:jc w:val="center"/>'
        f'<w:tblBorders>{borders}</w:tblBorders><w:tblLayout w:type="fixed"/>'
        '<w:tblLook w:firstRow="1" w:lastRow="0" w:firstColumn="0" w:lastColumn="0"'
        ' w:noHBand="0" w:noVBand="0"/></w:tblPr>'
        '<w:tblGrid>' + ''.join('<w:gridCol w:w="%d"/>' % w for w in grid) + '</w:tblGrid>'
        + ''.join(trs) + '</w:tbl>'
    )


# --------------------------------------------------------------------------
# document content
# --------------------------------------------------------------------------

TITLE = 'DeltaSeek: Toward Active Perception in Evolving Construction Environments'

AUTHORS = [
    ['Sanjay Acharjee^{1}',
     'Ph.D. Student, Dept. of Civil Eng.',
     'University of Texas at Arlington',
     'Arlington, TX, USA',
     'E-mail: abirkhan.ratul@uta.edu'],
    ['Md Nazmus Sakib, Ph.D.^{2*}',
     'Assistant Professor, Dept. of Civil Eng.',
     'University of Texas at Arlington',
     'Arlington, TX, USA',
     'E-mail: mdnazmus.sakib@uta.edu'],
]

ABSTRACT = [
    'Abstract—Construction environments evolve continuously: elements are '
    'installed, temporary objects appear and disappear, and surfaces that were '
    'visible last week are inaccessible today. Perception in such settings cannot '
    'be treated purely as a static mapping problem in which observations are '
    'gathered first and interpreted afterward, because large geometric change '
    'reduces the correspondence between observations taken at different times and '
    'makes registration progressively harder. This motivates an active '
    'formulation, in which the robot deliberately selects sensing configurations '
    'that resolve the current state of the environment.',

    'We present DeltaSeek, an initial framework toward active perception in '
    'evolving built environments. The broader objective is a robot that reasons '
    'about *where* to observe, *how* to observe, and ultimately *when* enough '
    'evidence has been gathered; as a first step we address a prerequisite common '
    'to all three, namely how far sensing embodiment constrains the observations '
    'available at all. We formalise the admissible observation set of an '
    'embodiment and evaluate a Husky A300 with a UR5e on an IFC-derived benchmark '
    'under chassis-mounted and wrist-mounted RGB-D configurations, scoring '
    'observations by geometric visibility and effort by drivable distance. In a '
    'room-scale scene with eight controlled changes spanning four observability '
    'conditions, exhaustive evaluation over 240 admissible base poses shows that '
    'two changes admit *no* chassis viewpoint whatsoever while the wrist camera '
    'resolves both, and that where both embodiments succeed the median drivable '
    'distance to first observation rises from 6.0 m to 14.2 m. The results '
    'separate two effects an active-perception system must treat differently—'
    '*sensing capability*, where embodiment decides whether an informative '
    'observation exists, and *acquisition cost*, where it exists but must be paid '
    'for in motion.',
]

KEYWORDS = ('Keywords—active perception, viewpoint planning, construction '
            'robotics, building information modelling, mobile manipulation')

FIG1_CAPTION = (
    'The study, in simulation. (a) A Husky A300 with a UR5e, carrying a chassis '
    'RGB-D camera and a lidar on the deck and an identical RGB-D camera at the '
    'wrist. (b) Hall B, clipped from a real Revit IFC export, with the eight '
    'injected changes. (c) Admissible observation sets: each sub-panel uses the '
    'base yaw that aims *its* sensor at the same target, since the arm rotates '
    'the wrist camera 90° off the base heading. Frusta are drawn to the '
    "target's depth; the change falls outside the chassis frustum and inside the "
    "wrist one. Renders and frusta come from the evaluation's camera poses.")

FIG2_CAPTION = (
    'Why exactly two changes admit no chassis viewpoint. At the 5 m range limit '
    'the chassis camera reaches 2.24 m and the wrist camera 4.63 m. The ceiling '
    'begins at 2.44 m, leaving a wrist-only band 0.20 m tall, in which the two '
    '*height* changes sit at 2.25 m. Both reaches are closed-form, and agree with '
    'a numeric sweep to 3 mm.')

FIG3_CAPTION = (
    '(a) Capability: |*Q*_{e}^{s}| as a share of admissible sensor poses for each '
    'change; two changes admit no chassis viewpoint at all. (b) Acquisition cost: '
    'drivable distance to first observation under a 25 m budget.')

REFERENCES = [
    'F. Bosché, “Automated recognition of 3D CAD model objects in laser '
    'scans and calculation of as-built dimensions for dimensional compliance '
    'control in construction,” *Advanced Engineering Informatics*, vol. 24, '
    'no. 1, pp. 107–118, 2010.',
    'Y. Turkan, F. Bosché, C. T. Haas, and R. Haas, “Automated progress '
    'tracking using 4D schedule and 3D sensing technologies,” *Automation in '
    'Construction*, vol. 22, pp. 414–421, 2012.',
    'M. Golparvar-Fard, F. Peña-Mora, and S. Savarese, “Automated '
    'progress monitoring using unordered daily construction photographs and '
    'IFC-based building information models,” *Journal of Computing in Civil '
    'Engineering*, vol. 29, no. 1, 2015.',
    'R. Bajcsy, “Active perception,” *Proceedings of the IEEE*, vol. 76, '
    'no. 8, pp. 966–1005, 1988.',
    'J. Aloimonos, I. Weiss, and A. Bandyopadhyay, “Active vision,” '
    '*International Journal of Computer Vision*, vol. 1, no. 4, pp. 333–356, '
    '1988.',
    'C. Connolly, “The determination of next best views,” in *Proc. IEEE '
    'Int. Conf. Robotics and Automation (ICRA)*, 1985, pp. 432–435.',
    'S. Isler, R. Sabzevari, J. Delmerico, and D. Scaramuzza, “An information '
    'gain formulation for active volumetric 3D reconstruction,” in *Proc. IEEE '
    'Int. Conf. Robotics and Automation (ICRA)*, 2016, pp. 3477–3484.',
    'A. Bircher, M. Kamel, K. Alexis, H. Oleynikova, and R. Siegwart, '
    '“Receding horizon ‘next-best-view’ planner for 3D '
    'exploration,” in *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*, '
    '2016, pp. 1462–1468.',
    'B. Yamauchi, “A frontier-based approach for autonomous exploration,” '
    'in *Proc. IEEE Int. Symp. Computational Intelligence in Robotics and '
    'Automation (CIRA)*, 1997, pp. 146–151.',
    'ISO 16739-1:2018, *Industry Foundation Classes (IFC) for data sharing in the '
    'construction and facility management industries*, ISO, Geneva, 2018.',
    'IfcOpenShell IFC toolkit and geometry engine. [Online]. Available: '
    'https://ifcopenshell.org',
]


def build_body(media: Media) -> str:
    out: list[str] = []
    add = out.append

    # --- title, authors -----------------------------------------------------
    add(para('papertitle', run(TITLE), SECT_TITLE))
    for index, block in enumerate(AUTHORS):
        last = index == len(AUTHORS) - 1
        add(text_para('Author', block[0]))
        for line in block[1:-1]:
            add(text_para('Affiliation', line))
        add(text_para('Affiliation', block[-1], SECT_AUTHORS if last else ''))

    # --- abstract -----------------------------------------------------------
    for text in ABSTRACT:
        add(text_para('Abstract', text))
    add(text_para('Keywords', KEYWORDS, SECT_2COL))

    # --- Fig. 1 spans the page ---------------------------------------------
    add(figure(media, 'fig_platform', 490, FIG1_CAPTION, SECT_1COL))

    # --- I. Introduction ----------------------------------------------------
    add(text_para('Heading1', 'Introduction'))
    add(text_para('BodyText',
        'A construction site is not a scene to be mapped once. Between two visits '
        'walls are erected, services are hung, pallets are delivered and consumed, '
        'and formwork is struck; the regions that changed are precisely the regions '
        'a verification system cares about. Robotic capture pipelines are '
        'nevertheless organised predominantly around passive accumulation: a '
        'coverage route is driven, point clouds or images are collected, registered '
        'to the model, and differenced offline [1], [2], [3]. Registration depends '
        'on correspondence between what was observed before and what is observed '
        'now, and large change is exactly the regime in which that correspondence '
        'becomes sparse: the information the robot most requires is the information '
        'its pipeline is least equipped to acquire.'))
    add(text_para('BodyText',
        'Active perception provides the alternative framing [4], [5]: sensor '
        'placement is treated as a decision that resolves a specific uncertainty. '
        'Applied to an evolving building, this amounts to asking where to look next '
        'in order to determine whether part of the model still describes reality, '
        'which requires answering, in sequence, where to observe, how to observe, '
        'and when sufficient evidence has been gathered.'))
    add(text_para('BodyText',
        'This paper takes a deliberately narrow first step. Before a planner can '
        "choose among sensing configurations, the set of observations the robot's "
        'embodiment makes available must be characterised. A mobile manipulator can '
        'place a camera almost anywhere in its reachable workspace; a fixed chassis '
        'sensor cannot. That freedom is commonly assumed valuable and seldom '
        'measured. Our contributions are a formulation separating the admissible '
        'observation set of an embodiment from the cost of reaching it; a '
        'reproducible IFC-derived benchmark that clips a real Revit export into a '
        'room-scale scene and injects controlled changes labelled by an '
        'observability class describing *why* they are hard to observe; and an '
        'exhaustive sensor-configuration study whose central finding is a '
        'distinction we believe generalises—embodiment partitions scene changes '
        'into those a configuration cannot observe at all, and those it can observe '
        'only at greater cost.'))

    # --- II. Related work ---------------------------------------------------
    add(text_para('Heading1', 'Related Work'))
    add(text_para('BodyText',
        '**As-built verification** against a building model is well established, '
        'from dimensional compliance checking [1] to progress tracking [2], [3]. '
        'Such systems consume data rather than plan it: the route is set by coverage '
        'or an operator, and the verification objective does not influence where the '
        'sensor goes.'))
    add(text_para('BodyText',
        '**Active perception** treats sensing as an action to be planned [4], [5]. '
        'Next-best-view methods maximise expected information for reconstruction '
        '[6], [7] or exploration [8], [9], typically under an unknown-space '
        'objective. Our setting differs: a strong prior exists, and the question '
        'concerns what has *changed*. Mounting the sensor on a manipulator enlarges '
        'the achievable viewpoint set, but the magnitude of that gain is rarely '
        'quantified.'))

    # --- III. Problem formulation ------------------------------------------
    add(text_para('Heading1', 'Problem Formulation'))
    add(text_para('BodyText',
        'Let *M* denote prior knowledge of the environment, such as a BIM model, a '
        'previous map, or a registered scan, and let *E*_{t} denote the unknown '
        'physical state at time *t*. A sensing configuration *q* consists of a '
        'mobile-base configuration *q*_{b} and an arm configuration *q*_{a}:'))
    add(equation(EQ1, '1'))
    add(text_para('BodyText',
        'A general active-perception system would select a feasible configuration by '
        'trading the expected task-relevant utility of a future observation against '
        'its acquisition cost:'))
    add(equation(EQ2, '2'))
    add(text_para('BodyText',
        'Here *Q* is the feasible sensing-action space, *B*_{t} denotes the evidence '
        'or belief accumulated so far, *U* is the expected utility of the '
        'observation for the current task, and *C* is the cost of acquiring it. The '
        'long-term DeltaSeek objective is to instantiate *U* using perceptual '
        'evidence such as RGB, depth, point-cloud completeness, registration '
        'confidence, or task-specific uncertainty, and to introduce a stopping rule '
        'for deciding when further sensing is unnecessary.'))
    add(text_para('BodyText',
        'The present paper deliberately isolates the geometric prerequisite for (2). '
        'For a scene change or observation target *e* and sensing configuration *q*, '
        'let *V*(*e*, *q*) ∈ [0, 1] be the visible supporting-surface fraction '
        'after range, frustum, incidence, and occlusion tests. An observation is '
        'considered geometrically admissible when'))
    add(equation(EQ3, '3'))
    add(text_para('BodyText',
        'For sensing embodiment *s* (chassis or wrist), define the admissible '
        'observation set'))
    add(equation(EQ4, '4'))
    add(text_para('BodyText',
        'The set *Q*_{e}^{s} provides the central decomposition studied in this '
        'paper. If *Q*_{e}^{s} is empty, the target lies outside the sensing '
        'capability of embodiment *s* and no routing policy can recover it. If '
        '*Q*_{e}^{s} is nonempty, the observation is feasible and the question '
        'becomes how much motion is required to reach one of its admissible '
        'configurations. With start configuration *q*_{0}, an idealised acquisition '
        'cost is'))
    add(equation(EQ5, '5'))
    add(text_para('BodyText',
        'where *d*_{nav} is collision-aware drivable distance; in planned runs the '
        'realised cost is the accumulated route length at which *e* is first '
        'observed. Fig. 3 visualises these two quantities directly: the left panel '
        'estimates the size of *Q*_{e}^{s}, the right panel the cost of reaching it.'))

    # --- IV. Benchmark ------------------------------------------------------
    add(text_para('Heading1', 'Benchmark'))
    add(text_para('Heading2', 'From IFC to a room-scale scene'))
    add(text_para('BodyText',
        'The benchmark derives its geometry from a Revit 2025 IFC2X3 [10] export of '
        'a single-storey building, 44.5 × 19.0 × 4.5 m, comprising 133 '
        'products that convert without failure through IfcOpenShell [11]. Each '
        'product is approximated by a box oriented in its own placement frame. '
        'Orientation is not a refinement: the export sits on a site grid rotated '
        '0.90° from the world axes, and a world-aligned box expands a 0.203 m '
        'wall to 0.82 m—a fourfold error on the dimension deciding whether the '
        'robot fits between two walls.'))
    add(text_para('BodyText',
        'A storey is an unsuitable unit for studying observation—one wall entity '
        'spans the full 39 m of a facade—so we clip the manifest to a rectangle, '
        "cutting in each element's own frame to preserve the rotation. The resulting "
        'scene, Hall B (Fig. 1b), measures 7.0 × 12.5 m and contains twelve '
        'modelled elements plus two added fit-out units, since the export has no '
        'furniture and both occlusion and height-limited observability need objects '
        'to stand behind or on top of.'))
    add(text_para('Heading2', 'Observation model'))
    add(text_para('BodyText',
        'The visibility term *V*(*e*, *q*) of (3) is evaluated geometrically rather '
        "than from rendered imagery. An element's surface is sampled, and a sample "
        'counts when it lies within the view frustum, faces the camera within a '
        '75° incidence limit, and is unoccluded. Each change type carries its '
        'own supporting surface: an unmodelled object is supported by seeing the '
        'object, a displaced element by seeing it in its new position or its '
        'modelled volume empty. The threshold τ_{v} = 0.05 stands in for '
        'detector sensitivity. The model assumes perfect recognition and '
        'localisation, so it measures *viewpoint quality* rather than perception '
        'robustness; in exchange it is fast enough to evaluate exhaustively over '
        '*Q*_{s}, which rendering is not.'))
    add(text_para('Heading2', 'Acquisition cost'))
    add(text_para('BodyText',
        'The distance *d*_{nav} of (5) is a Dijkstra search over an occupancy grid '
        "inflated by the platform's Nav2 footprint (0.606 m circumscribed radius, "
        '0.06 m resolution). The choice matters: on the storey-scale scene drivable '
        'distance averages 1.8× the straight line and reaches 11×, and a '
        'straight-line surrogate would let a robot pass through walls to reach a '
        'viewpoint.'))
    add(text_para('Heading2', 'Observability classes'))
    add(text_para('BodyText',
        'The type of a change describes what moved; it says nothing about whether a '
        'sensor can observe it. We therefore label each change by *why* it is hard '
        'to observe: *trivial*, a large unmodelled object in open floor space; '
        "*height*, above the chassis camera's sightline; *incidence*, resolvable "
        "only from an oblique angle; and *occluded*, in another object's shadow from "
        'most base poses.'))
    add(text_para('BodyText',
        'Eight changes were hand-placed in Hall B, two per class, in a committed '
        'scenario rather than sampled, so that each is explainable from geometry and '
        'the study is exactly reproducible. The *trivial* pair is a control: a '
        'configuration that misses it is demonstrably broken.'))

    # --- V. Sensing configurations -----------------------------------------
    add(text_para('Heading1', 'Sensing Configurations'))
    add(text_para('BodyText',
        'The platform is a Clearpath Husky A300 with an AMP enclosure and a UR5e, '
        'generated from a single robot.yaml so simulation and hardware share one '
        'description. It carries two RGB-D cameras of the *same* model, so the '
        'comparison isolates where a camera is mounted rather than how capable it '
        'is. The **chassis** embodiment places the camera on the enclosure deck at '
        '(0.460, 0.000, 0.418) m in the base frame, pitched 0.17 rad down; its pose '
        'is a function of *q*_{b} alone. The **wrist** embodiment is eye-in-hand, '
        'its pose obtained by forward kinematics over the generated description '
        'across five inspection postures *q*_{a}, verified against the running '
        "simulation's transform tree to machine precision."))
    add(text_para('BodyText',
        'The base configurations *q*_{b} lie on a 1.0 m grid of navigable free space '
        'inside the room at four yaw bins 90° apart, giving |*Q*_{b}| = 240 '
        'admissible base poses from 60 positions; both embodiments share this set, '
        'so the comparison is not confounded by base sampling. Each base pose '
        'combines with the five arm postures, so |*Q*_{wrist}| = 1200 distinct camera '
        'poses while |*Q*_{chassis}| = 240, the chassis camera being invariant to '
        '*q*_{a}. In the *both* configuration an element is observed if either '
        'sensor observes it.'))
    add(figure(media, 'fig_reach', 200, FIG2_CAPTION))

    # --- Fig. 3 spans the page ---------------------------------------------
    add(text_para(None, '', SECT_2COL))
    add(figure(media, 'fig_result', 400, FIG3_CAPTION, SECT_1COL))

    # --- VI. Results --------------------------------------------------------
    add(text_para('Heading1', 'Results'))
    add(text_para('Heading2', 'Capability: the structure of *Q*_{e}^{s}'))
    add(text_para('BodyText',
        'Table I reports what each configuration observed within a 25 m budget; '
        'Fig. 3(a) reports the underlying capability.'))
    add(text_para('tablehead', 'Changes observed, by observability class'))
    add(results_table())
    add(text_para(None, ''))
    add(text_para('BodyText',
        'The result of interest is not the 5/8 figure. A single planned run '
        'conflates *Q*_{e}^{s} = ∅ with the case in which one route failed to '
        'reach an admissible pose, so we separate the two by evaluating '
        '*V*(*e*, *q*) at every *q* ∈ *Q*_{s}. For two changes—both of '
        'class *height*—*Q*_{e}^{chassis} is empty, at zero visible fraction '
        'across all 240 chassis poses; this is a property of the embodiment. A third '
        'change that the chassis-only run missed satisfies |*Q*_{e}^{chassis}| = 14: '
        'the planner did not visit an admissible pose within budget, which is a '
        'routing outcome, not evidence for the manipulator. The distinction is not '
        'merely formal—repositioning the arm on the deck during development '
        'moved the planned chassis result from 6/8 to 5/8 while the capability '
        'figure did not move at all.'))
    add(text_para('BodyText',
        'Fig. 2 explains why only the *height* class produces an empty admissible '
        'set. For a camera of pitch *p* at height *h* with vertical field of view '
        '*v*, the highest world point inside its frustum at range *r* is '
        '*h* + *r*(cos *p* tan(*v*/2) − sin *p*), with no yaw term: yaw rotates '
        'about the vertical axis and cannot alter the vertical extent. At *r* = 5 m '
        'this gives 2.24 m for the chassis camera and 4.63 m for the wrist, against '
        'a ceiling at 2.44 m. The wrist-only band is therefore 0.20 m tall, and the '
        'two *height* changes were placed within it at 2.25 m.'))
    add(text_para('Heading2',
        'Acquisition cost: *C*_{s}(*e*) where both embodiments succeed'))
    add(text_para('BodyText',
        'The *occluded* and *incidence* pairs were designed to require the '
        'manipulator and did not. The crate behind the pallet stack satisfies '
        '|*Q*_{e}^{chassis}| = 16 of 240—hidden from 93% of chassis poses, as '
        'intended—but a planner needs only one of those sixteen. Where both '
        'embodiments succeed the difference appears in *C*_{s}(*e*), not in '
        'detections (Fig. 3b): the median drivable distance to first observation is '
        '6.0 m for the wrist and 14.2 m for the chassis, a factor of 2.4. Occlusion, '
        'the intuitive case for a manipulator, is in this environment a routing cost '
        'paid in metres rather than a sensing limit.'))

    # --- VII. Discussion ----------------------------------------------------
    add(text_para('Heading1', 'Discussion'))
    add(text_para('BodyText',
        'Two effects follow, and an active-perception system must treat them '
        'differently. **Sensing capability** is binary and fixed by embodiment: when '
        '*Q*_{e}^{s} = ∅ no planning policy recovers the change, and the utility '
        'term *U* of (2) is identically zero over *Q*_{s}. **Acquisition cost** is '
        'continuous and fixed by the environment: the information exists, and the '
        'question is the value of *C*_{s}(*e*). This is where viewpoint planning has '
        'leverage, and where the 6.0 m against 14.2 m difference lives. Conflating '
        'the two is easy, since a single run reports only “observed” or '
        '“not observed” and an expensive change is then indistinguishable '
        'from an impossible one.'))
    add(text_para('BodyText',
        'The capability boundary is also a design variable: the wrist-only band in '
        'Hall B is 0.20 m because the chassis camera sits at 0.418 m pitched '
        '0.17 rad down, levelling it shrinks the band toward zero, and a taller '
        'space widens it. The useful output is therefore not the claim that the '
        'manipulator is worth two changes, but a method for locating that boundary '
        'before a platform design is committed.'))

    # --- VIII. Limitations --------------------------------------------------
    add(text_para('Heading1', 'Limitations and Future Work'))
    add(text_para('BodyText',
        'The study is narrow: one room, one ceiling height, eight changes. The '
        '0.20 m band is a property of this geometry, and a trend requires rooms of '
        'differing height. Detection is geometric—no detector, no renderer, no '
        'noise—so the reported quantities describe viewpoint quality, not '
        'whether a perception stack would recover the change. Elements are boxes, '
        'discarding the 48 opening entities in the source model, so Hall B is a '
        'sealed room the robot must be spawned inside. The height margin is thin: '
        'the *height* pair clears the chassis ceiling by 11.8 mm, so levelling or '
        'raising that camera would close the gap—the design-variable '
        'observation above, restated as a caution. Finally, the framework does not '
        'yet answer its third question: deciding *when* enough evidence has been '
        'gathered requires a graded belief *B*_{t} and a stopping rule, not binary '
        'detection.'))

    # --- IX. Conclusion -----------------------------------------------------
    add(text_para('Heading1', 'Conclusion'))
    add(text_para('BodyText',
        'DeltaSeek is a first step toward robots that plan perception in '
        'environments that do not hold still. We formalised the contribution of a '
        "mobile manipulator's embodiment as the structure of the admissible "
        'observation set *Q*_{e}^{s} and the cost *C*_{s}(*e*) of reaching it, and '
        'measured both on an IFC-derived benchmark. The answer is sharper and '
        'smaller than the usual assumption: two of eight changes lie outside the '
        "chassis camera's admissible set entirely, and the rest cost roughly "
        '2.4× the travel. A robot that is one day to decide when it has seen '
        'enough must first distinguish a view it cannot obtain from one it has not '
        'yet paid for.'))

    # --- references ---------------------------------------------------------
    add(text_para('Heading5', 'References'))
    for entry in REFERENCES:
        add(text_para('references', entry))

    return ''.join(out)


NAMESPACES = ' '.join([
    'xmlns:w="http://purl.oclc.org/ooxml/wordprocessingml/main"',
    'xmlns:r="http://purl.oclc.org/ooxml/officeDocument/relationships"',
    'xmlns:m="http://purl.oclc.org/ooxml/officeDocument/math"',
    'xmlns:wp="http://purl.oclc.org/ooxml/drawingml/wordprocessingDrawing"',
    'xmlns:a="http://purl.oclc.org/ooxml/drawingml/main"',
    'xmlns:pic="http://purl.oclc.org/ooxml/drawingml/picture"',
])


def render_figures() -> None:
    out = FIGURES / 'png'
    out.mkdir(exist_ok=True)
    for name in ('fig_platform', 'fig_reach', 'fig_result'):
        target = out / f'{name}.png'
        source = FIGURES / f'{name}.pdf'
        if not target.exists() or target.stat().st_mtime < source.stat().st_mtime:
            subprocess.run(['pdftocairo', '-png', '-r', '300', '-singlefile',
                            str(source), str(out / name)], check=True)


EXPECTED_PAGE = {'a4': (595, 842), 'letter': (612, 792)}


def verify(output: Path) -> None:
    """Render the package and check it opens at the intended page size.

    Word is not available here, so the document was previously shipped
    unrendered: well-formed XML referencing styles that exist proves neither
    that it opens nor that it paginates. LibreOffice renders it, which is a
    weaker check than Word but catches a package that is broken outright.
    """
    if not shutil.which('soffice'):
        print('  (soffice not found: rendering not verified)')
        return
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(['soffice', '--headless', '--convert-to', 'pdf',
                        '--outdir', tmp, str(output)],
                       capture_output=True, timeout=300, check=True)
        pdf = Path(tmp) / (output.stem + '.pdf')
        if not pdf.exists():
            raise SystemExit(f'  FAILED: {output.name} did not render')
        info = subprocess.run(['pdfinfo', str(pdf)], capture_output=True,
                              text=True, timeout=60).stdout
        pages = int(re.search(r'Pages:\s+(\d+)', info).group(1))
        size = re.search(r'Page size:\s+([\d.]+) x ([\d.]+)', info)
        got = (round(float(size.group(1))), round(float(size.group(2))))
        want = EXPECTED_PAGE[PAPER]
        ok = all(abs(a - b) <= 1 for a, b in zip(got, want))
        print(f'  rendered {pages} pages at {got[0]} x {got[1]} pt'
              f'{"" if ok else f"  MISMATCH, expected {want}"}')
        if not ok:
            raise SystemExit('page size does not match --paper')


def main() -> None:
    global PAPER
    argv = [a for a in sys.argv[1:] if not a.startswith('--')]
    for flag in sys.argv[1:]:
        if flag.startswith('--paper='):
            PAPER = flag.split('=', 1)[1]
            if PAPER not in PAPER_SIZES:
                raise SystemExit(f'--paper must be one of {sorted(PAPER_SIZES)}')
            _sections()
    template = Path(argv[0]) if argv else ROOT / 'conference-template-a4.docx'
    output = (Path(argv[1]) if len(argv) > 1
              else ROOT / 'ieeeconf' / f'deltaseek_ieee_{PAPER}.docx')

    render_figures()

    media = Media()
    body = build_body(media)
    # the trailing sectPr belongs to w:body itself and governs the final section
    document = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
                f'<w:document {NAMESPACES}><w:body>{body}{SECT_2COL}'
                '</w:body></w:document>')

    with zipfile.ZipFile(template) as zin:
        names = zin.namelist()
        parts = {n: zin.read(n) for n in names}

    parts['word/document.xml'] = document.encode('utf-8')

    # relationships for the images
    rels = parts['word/_rels/document.xml.rels'].decode('utf-8')
    additions = ''.join(
        f'<Relationship Id="{rid}" '
        'Type="http://purl.oclc.org/ooxml/officeDocument/relationships/image" '
        f'Target="media/{path.name}"/>'
        for rid, path in media.items)
    rels = rels.replace('</Relationships>', additions + '</Relationships>')
    parts['word/_rels/document.xml.rels'] = rels.encode('utf-8')

    content_types = parts['[Content_Types].xml'].decode('utf-8')
    if 'Extension="png"' not in content_types:
        content_types = content_types.replace(
            '<Default Extension="xml"',
            '<Default Extension="png" ContentType="image/png"/><Default Extension="xml"')
    parts['[Content_Types].xml'] = content_types.encode('utf-8')

    for _, path in media.items:
        parts[f'word/media/{path.name}'] = path.read_bytes()

    core = parts['docProps/core.xml'].decode('utf-8')
    core = re.sub(r'(<dc:title>).*?(</dc:title>)', r'\g<1>' + TITLE + r'\g<2>', core)
    core = re.sub(r'(<dc:creator>).*?(</dc:creator>)',
                  r'\g<1>Sanjay Acharjee; Md Nazmus Sakib\g<2>', core)
    parts['docProps/core.xml'] = core.encode('utf-8')

    with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as zout:
        for name, data in parts.items():
            zout.writestr(name, data)

    print(f'wrote {output} ({output.stat().st_size / 1024:.0f} KiB)')
    verify(output)


if __name__ == '__main__':
    main()
