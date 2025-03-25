import webots_ros2_suv.lib.gis_args as gis_args
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, String, Float
from sqlalchemy.orm import sessionmaker
from geoalchemy2 import Geometry


Base = declarative_base()
engine = create_engine(f'postgresql://{gis_args.user}:{gis_args.passwd}@{gis_args.host}/{gis_args.dbname}', echo=False)


class FieldChank(Base):
    __tablename__ = 'field_chank'
    id = Column(Integer, primary_key=True)
    crop_name = Column(String)
    polygon = Column(Geometry('POLYGON'))
    position_x = Column(Float)
    position_y = Column(Float)
    irrigation_degree = Column(Float)


try:
    FieldChank.__table__.create(engine)
except:
    pass


Session = sessionmaker(bind=engine)
session = Session()

# chanks = session.query(FieldChank).all()
# for chank in chanks:
    
