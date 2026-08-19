"""Create the tiny deterministic IFC4 model used by DeltaSeek smoke tests."""

import argparse
from pathlib import Path
import uuid

import ifcopenshell.api.aggregate
import ifcopenshell.api.context
import ifcopenshell.api.geometry
import ifcopenshell.api.project
import ifcopenshell.api.root
import ifcopenshell.api.spatial
import ifcopenshell.api.unit
import ifcopenshell.guid


NAMESPACE = uuid.UUID('ddbf47ea-df53-4756-b14d-d4a619ec8e2f')


def _guid(name):
    return ifcopenshell.guid.compress(uuid.uuid5(NAMESPACE, name).hex)


def _box_vertices(center, size):
    x0, y0, z0 = [c - s / 2.0 for c, s in zip(center, size)]
    x1, y1, z1 = [c + s / 2.0 for c, s in zip(center, size)]
    return [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]


BOX_FACES = [
    (0, 3, 2, 1),
    (4, 5, 6, 7),
    (0, 1, 5, 4),
    (1, 2, 6, 5),
    (2, 3, 7, 6),
    (3, 0, 4, 7),
]


def create_demo(path):
    model = ifcopenshell.api.project.create_file(version='IFC4')
    project = ifcopenshell.api.root.create_entity(
        model, ifc_class='IfcProject', name='DeltaSeek Demo')
    project.GlobalId = _guid('project')
    ifcopenshell.api.unit.assign_unit(model)
    context = ifcopenshell.api.context.add_context(
        model, context_type='Model')
    body = ifcopenshell.api.context.add_context(
        model,
        context_type='Model',
        context_identifier='Body',
        target_view='MODEL_VIEW',
        parent=context,
    )

    site = ifcopenshell.api.root.create_entity(
        model, ifc_class='IfcSite', name='Demo Site')
    building = ifcopenshell.api.root.create_entity(
        model, ifc_class='IfcBuilding', name='Demo Building')
    storey = ifcopenshell.api.root.create_entity(
        model, ifc_class='IfcBuildingStorey', name='Ground Floor')
    site.GlobalId = _guid('site')
    building.GlobalId = _guid('building')
    storey.GlobalId = _guid('storey')
    ifcopenshell.api.aggregate.assign_object(
        model, products=[site], relating_object=project)
    ifcopenshell.api.aggregate.assign_object(
        model, products=[building], relating_object=site)
    ifcopenshell.api.aggregate.assign_object(
        model, products=[storey], relating_object=building)

    elements = [
        ('IfcSlab', 'Ground slab', [0.0, 0.0, -0.1], [18.0, 12.0, 0.2]),
        ('IfcWall', 'North wall', [0.0, 5.5, 1.5], [18.0, 0.2, 3.0]),
        ('IfcColumn', 'Column A', [-3.0, -2.0, 1.5], [0.45, 0.45, 3.0]),
        ('IfcColumn', 'Column B', [-3.0, 2.5, 1.5], [0.45, 0.45, 3.0]),
        ('IfcFlowSegment', 'Suspended pipe', [2.0, 0.0, 2.2], [0.24, 5.0, 0.24]),
        ('IfcBeam', 'Cross beam', [-1.5, 1.5, 2.65], [3.0, 0.22, 0.3]),
        ('IfcFlowSegment', 'Rectangular duct', [3.5, 2.0, 2.35], [3.2, 0.5, 0.4]),
    ]
    for ifc_class, name, center, size in elements:
        product = ifcopenshell.api.root.create_entity(
            model, ifc_class=ifc_class, name=name)
        product.GlobalId = _guid(name)
        representation = ifcopenshell.api.geometry.add_mesh_representation(
            model,
            context=body,
            vertices=[_box_vertices(center, size)],
            faces=[BOX_FACES],
        )
        ifcopenshell.api.geometry.assign_representation(
            model, product=product, representation=representation)
        ifcopenshell.api.geometry.edit_object_placement(
            model, product=product)
        ifcopenshell.api.spatial.assign_container(
            model, products=[product], relating_structure=storey)

    path.parent.mkdir(parents=True, exist_ok=True)
    model.write(str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    create_demo(args.output)
    print(args.output)


if __name__ == '__main__':
    main()
