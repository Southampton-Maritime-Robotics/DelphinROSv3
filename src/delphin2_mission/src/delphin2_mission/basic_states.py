#=========================
# A list of available basic states
#-------------------------

from state_actions                          import actions
from state_initialise                       import Initialise
from state_stop                             import Stop
from state_goToDepth                        import GoToDepth
from state_goToAltitude                     import GoToAltitude
from state_goToHeading                      import GoToHeading
from state_goForwards                       import GoForwards
from state_goToAltitude                     import GoToAltitude
from state_goTurning                        import GoTurning
from state_pathFollowingLOS                 import pathFollowingLOS
from state_verboseLocation                  import verboseLocation
from state_reviseWaypoints                  import reviseWaypoints
from state_waitForGPS                       import waitForGPS
from state_goJibbing                        import GoJibbing
