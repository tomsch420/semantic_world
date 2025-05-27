from semantic_world.orm.model import TransformationMatrixType

from sqlalchemy import Column, Float, ForeignKey, Integer, MetaData, String, Table
from sqlalchemy.orm import RelationshipProperty, registry, relationship
import semantic_world.connections
import semantic_world.geometry
import semantic_world.prefixed_name
import semantic_world.world

metadata = MetaData()


t_Color = Table(
    'Color', metadata,
    Column('id', Integer, primary_key=True),
    Column('R', Float, nullable=False),
    Column('G', Float, nullable=False),
    Column('B', Float, nullable=False),
    Column('A', Float, nullable=False)
)

t_PrefixedName = Table(
    'PrefixedName', metadata,
    Column('id', Integer, primary_key=True),
    Column('name', String, nullable=False),
    Column('prefix', String)
)

t_Scale = Table(
    'Scale', metadata,
    Column('id', Integer, primary_key=True),
    Column('x', Float, nullable=False),
    Column('y', Float, nullable=False),
    Column('z', Float, nullable=False)
)

t_UnitVector = Table(
    'UnitVector', metadata,
    Column('id', Integer, primary_key=True),
    Column('x', Float, nullable=False),
    Column('y', Float, nullable=False),
    Column('z', Float, nullable=False)
)

t_WorldEntity = Table(
    'WorldEntity', metadata,
    Column('id', Integer, primary_key=True),
    Column('polymorphic_type', String)
)

t_Body = Table(
    'Body', metadata,
    Column('id', ForeignKey('WorldEntity.id'), primary_key=True),
    Column('name_id', ForeignKey('PrefixedName.id'), nullable=False)
)

t_View = Table(
    'View', metadata,
    Column('id', ForeignKey('WorldEntity.id'), primary_key=True)
)

t_Connection = Table(
    'Connection', metadata,
    Column('id', ForeignKey('WorldEntity.id'), primary_key=True),
    Column('parent_id', ForeignKey('Body.id'), nullable=False),
    Column('child_id', ForeignKey('Body.id'), nullable=False),
    Column('origin', TransformationMatrixType, nullable=False)
)

t_Shape = Table(
    'Shape', metadata,
    Column('id', Integer, primary_key=True),
    Column('origin', TransformationMatrixType, nullable=False),
    Column('body_visual_id', ForeignKey('Body.id')),
    Column('body_collision_id', ForeignKey('Body.id')),
    Column('polymorphic_type', String)
)

t_ActiveConnection = Table(
    'ActiveConnection', metadata,
    Column('id', ForeignKey('Connection.id'), primary_key=True)
)

t_FixedConnection = Table(
    'FixedConnection', metadata,
    Column('id', ForeignKey('Connection.id'), primary_key=True)
)

t_Mesh = Table(
    'Mesh', metadata,
    Column('id', ForeignKey('Shape.id'), primary_key=True),
    Column('filename', String, nullable=False),
    Column('scale_id', ForeignKey('Scale.id'), nullable=False)
)

t_PassiveConnection = Table(
    'PassiveConnection', metadata,
    Column('id', ForeignKey('Connection.id'), primary_key=True)
)

t_Primitive = Table(
    'Primitive', metadata,
    Column('id', ForeignKey('Shape.id'), primary_key=True),
    Column('color_id', ForeignKey('Color.id'), nullable=False)
)

t_Box = Table(
    'Box', metadata,
    Column('id', ForeignKey('Primitive.id'), primary_key=True),
    Column('scale_id', ForeignKey('Scale.id'), nullable=False)
)

t_Cylinder = Table(
    'Cylinder', metadata,
    Column('id', ForeignKey('Primitive.id'), primary_key=True),
    Column('width', Float, nullable=False),
    Column('height', Float, nullable=False)
)

t_DegreeOfFreedom = Table(
    'DegreeOfFreedom', metadata,
    Column('id', ForeignKey('WorldEntity.id'), primary_key=True),
    Column('name_id', ForeignKey('PrefixedName.id'), nullable=False),
    Column('state_idx', Integer, nullable=False),
    Column('passiveconnection_passive_dofs_id', ForeignKey('PassiveConnection.id')),
    Column('activeconnection_active_dofs_id', ForeignKey('ActiveConnection.id'))
)

t_Sphere = Table(
    'Sphere', metadata,
    Column('id', ForeignKey('Primitive.id'), primary_key=True),
    Column('radius', Float, nullable=False)
)

t_Connection6DoF = Table(
    'Connection6DoF', metadata,
    Column('id', ForeignKey('PassiveConnection.id'), primary_key=True),
    Column('x_id', ForeignKey('DegreeOfFreedom.id'), nullable=False),
    Column('y_id', ForeignKey('DegreeOfFreedom.id'), nullable=False),
    Column('z_id', ForeignKey('DegreeOfFreedom.id'), nullable=False),
    Column('qx_id', ForeignKey('DegreeOfFreedom.id'), nullable=False),
    Column('qy_id', ForeignKey('DegreeOfFreedom.id'), nullable=False),
    Column('qz_id', ForeignKey('DegreeOfFreedom.id'), nullable=False),
    Column('qw_id', ForeignKey('DegreeOfFreedom.id'), nullable=False)
)

t_PrismaticConnection = Table(
    'PrismaticConnection', metadata,
    Column('id', ForeignKey('ActiveConnection.id'), primary_key=True),
    Column('axis_id', ForeignKey('UnitVector.id'), nullable=False),
    Column('multiplier', Float, nullable=False),
    Column('offset', Float, nullable=False),
    Column('dof_id', ForeignKey('DegreeOfFreedom.id'), nullable=False)
)

t_RevoluteConnection = Table(
    'RevoluteConnection', metadata,
    Column('id', ForeignKey('ActiveConnection.id'), primary_key=True),
    Column('axis_id', ForeignKey('UnitVector.id'), nullable=False),
    Column('multiplier', Float, nullable=False),
    Column('offset', Float, nullable=False),
    Column('dof_id', ForeignKey('DegreeOfFreedom.id'), nullable=False)
)

mapper_registry = registry(metadata=metadata)

m_Shape = mapper_registry.map_imperatively(semantic_world.geometry.Shape, t_Shape, properties = dict(origin=t_Shape.c.origin), polymorphic_on = "polymorphic_type", polymorphic_identity = "Shape")

m_WorldEntity = mapper_registry.map_imperatively(semantic_world.world.WorldEntity, t_WorldEntity, polymorphic_on = "polymorphic_type", polymorphic_identity = "WorldEntity")

m_UnitVector = mapper_registry.map_imperatively(semantic_world.connections.UnitVector, t_UnitVector, )

m_Color = mapper_registry.map_imperatively(semantic_world.geometry.Color, t_Color, )

m_Scale = mapper_registry.map_imperatively(semantic_world.geometry.Scale, t_Scale, )

m_PrefixedName = mapper_registry.map_imperatively(semantic_world.prefixed_name.PrefixedName, t_PrefixedName, )

m_Primitive = mapper_registry.map_imperatively(semantic_world.geometry.Primitive, t_Primitive, properties = dict(color=relationship('Color',foreign_keys=[t_Primitive.c.color_id])), polymorphic_identity = "Primitive", inherits = m_Shape)

m_Mesh = mapper_registry.map_imperatively(semantic_world.geometry.Mesh, t_Mesh, properties = dict(scale=relationship('Scale',foreign_keys=[t_Mesh.c.scale_id])), polymorphic_identity = "Mesh", inherits = m_Shape)

m_Connection = mapper_registry.map_imperatively(semantic_world.world.Connection, t_Connection, properties = dict(parent=relationship('Body',foreign_keys=[t_Connection.c.parent_id]), 
child=relationship('Body',foreign_keys=[t_Connection.c.child_id]), 
origin=t_Connection.c.origin), polymorphic_identity = "Connection", inherits = m_WorldEntity)

m_View = mapper_registry.map_imperatively(semantic_world.world.View, t_View, polymorphic_identity = "View", inherits = m_WorldEntity)

m_Body = mapper_registry.map_imperatively(semantic_world.world.Body, t_Body, properties = dict(name=relationship('PrefixedName',foreign_keys=[t_Body.c.name_id]), 
visual=relationship('Shape',foreign_keys=[t_Shape.c.body_visual_id]), 
collision=relationship('Shape',foreign_keys=[t_Shape.c.body_collision_id])), polymorphic_identity = "Body", inherits = m_WorldEntity)

m_DegreeOfFreedom = mapper_registry.map_imperatively(semantic_world.world.DegreeOfFreedom, t_DegreeOfFreedom, properties = dict(name=relationship('PrefixedName',foreign_keys=[t_DegreeOfFreedom.c.name_id])), polymorphic_identity = "DegreeOfFreedom", inherits = m_WorldEntity)

m_Cylinder = mapper_registry.map_imperatively(semantic_world.geometry.Cylinder, t_Cylinder, polymorphic_identity = "Cylinder", inherits = m_Primitive)

m_Box = mapper_registry.map_imperatively(semantic_world.geometry.Box, t_Box, properties = dict(scale=relationship('Scale',foreign_keys=[t_Box.c.scale_id])), polymorphic_identity = "Box", inherits = m_Primitive)

m_Sphere = mapper_registry.map_imperatively(semantic_world.geometry.Sphere, t_Sphere, polymorphic_identity = "Sphere", inherits = m_Primitive)

m_PassiveConnection = mapper_registry.map_imperatively(semantic_world.connections.PassiveConnection, t_PassiveConnection, properties = dict(passive_dofs=relationship('DegreeOfFreedom',foreign_keys=[t_DegreeOfFreedom.c.passiveconnection_passive_dofs_id])), polymorphic_identity = "PassiveConnection", inherits = m_Connection)

m_ActiveConnection = mapper_registry.map_imperatively(semantic_world.connections.ActiveConnection, t_ActiveConnection, properties = dict(active_dofs=relationship('DegreeOfFreedom',foreign_keys=[t_DegreeOfFreedom.c.activeconnection_active_dofs_id])), polymorphic_identity = "ActiveConnection", inherits = m_Connection)

m_FixedConnection = mapper_registry.map_imperatively(semantic_world.connections.FixedConnection, t_FixedConnection, polymorphic_identity = "FixedConnection", inherits = m_Connection)

m_Connection6DoF = mapper_registry.map_imperatively(semantic_world.connections.Connection6DoF, t_Connection6DoF, properties = dict(x=relationship('DegreeOfFreedom',foreign_keys=[t_Connection6DoF.c.x_id]), 
y=relationship('DegreeOfFreedom',foreign_keys=[t_Connection6DoF.c.y_id]), 
z=relationship('DegreeOfFreedom',foreign_keys=[t_Connection6DoF.c.z_id]), 
qx=relationship('DegreeOfFreedom',foreign_keys=[t_Connection6DoF.c.qx_id]), 
qy=relationship('DegreeOfFreedom',foreign_keys=[t_Connection6DoF.c.qy_id]), 
qz=relationship('DegreeOfFreedom',foreign_keys=[t_Connection6DoF.c.qz_id]), 
qw=relationship('DegreeOfFreedom',foreign_keys=[t_Connection6DoF.c.qw_id])), polymorphic_identity = "Connection6DoF", inherits = m_PassiveConnection)

m_PrismaticConnection = mapper_registry.map_imperatively(semantic_world.connections.PrismaticConnection, t_PrismaticConnection, properties = dict(axis=relationship('UnitVector',foreign_keys=[t_PrismaticConnection.c.axis_id]), 
dof=relationship('DegreeOfFreedom',foreign_keys=[t_PrismaticConnection.c.dof_id])), polymorphic_identity = "PrismaticConnection", inherits = m_ActiveConnection)

m_RevoluteConnection = mapper_registry.map_imperatively(semantic_world.connections.RevoluteConnection, t_RevoluteConnection, properties = dict(axis=relationship('UnitVector',foreign_keys=[t_RevoluteConnection.c.axis_id]), 
dof=relationship('DegreeOfFreedom',foreign_keys=[t_RevoluteConnection.c.dof_id])), polymorphic_identity = "RevoluteConnection", inherits = m_ActiveConnection)
