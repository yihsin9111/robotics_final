from .move import moveRoute
from .turn import turnRoute
from .shoot import shootRoute
from .shooter import shooterRoute
from .load import loadRoute
from .rise import riseRoute
from .shooter_rise import shooterRiseRoute
from .turn_camera import turnCameraRoute

Routes = dict(
    MOVE=moveRoute,
    TURN=turnRoute,
    SHOOTER=shooterRoute,
    SHOOT=shootRoute,
    LOAD=loadRoute,
    RISE=riseRoute,
    SHOOTER_RISE=shooterRiseRoute,
    TURN_CAMERA=turnCameraRoute,
)
