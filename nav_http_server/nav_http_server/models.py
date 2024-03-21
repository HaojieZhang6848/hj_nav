from pydantic import BaseModel, field_validator
from typing import Union, Optional

class NavRequest(BaseModel):
    x: Optional[Union[float, int]]
    y: Optional[Union[float, int]]
    theta: Optional[Union[float, int]]
    
    @field_validator('x')
    @classmethod
    def x_must_be_valid(cls, value):
        if value is None:
            raise ValueError('x must be provided')
        return value
    
    @field_validator('y')
    @classmethod
    def y_must_be_valid(cls, value):
        if value is None:
            raise ValueError('y must be provided')
        return value
    
    @field_validator('theta')
    @classmethod
    def theta_must_be_valid(cls, value):
        if value is None:
            raise ValueError('theta must be provided')
        return value
    
class InitialPoseRequest(BaseModel):
    x: Optional[Union[float, int]]
    y: Optional[Union[float, int]]
    theta: Optional[Union[float, int]]
    
    @field_validator('x')
    @classmethod
    def x_must_be_valid(cls, value):
        if value is None:
            raise ValueError('x must be provided')
        return value
    
    @field_validator('y')
    @classmethod
    def y_must_be_valid(cls, value):
        if value is None:
            raise ValueError('y must be provided')
        return value
    
    @field_validator('theta')
    @classmethod
    def theta_must_be_valid(cls, value):
        if value is None:
            raise ValueError('theta must be provided')
        return value