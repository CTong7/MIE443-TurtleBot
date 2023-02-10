#include "wall_follow.h"

void follow_wall(){
    // Detect an obstacle inside acquisition range (1.2 m)

    // Drive towards the obstacle (get within 0.6 m of it, obstacle should be in middle of our camera's FOV)

    // Choose a direction (ex: always left) and turn

    // WHILE:
    //// Look out for obstacles in the way and if the wall is ending soon.

    // Check left edge of FOV for obstacles

    // Check left edge of FOV for edge/end of the wall.

    //// If the way is not clear, follow that obstacle instead (becomes new "wall")

    // If obstacle, "acquire" it and drive towards it (0.6m, center of FOV).
    

    //// If the way is clear and the wall seems to continue, drive forward 1 meter.

    // Drive towards wall

    //// Turn and back off from wall, reassess while loop. (remember you can drive backwards)

    // Drive away from wall until 0.6m away again.

    //// If the way is clear and wall seems to end soon, still drive forward but less.

    // Drive forward, use your FOV to determine where relative to the robot the wall is ending

    // Reach the end of the wall.

    //// Execute search for new obstacle afterwards.

    // Random walk until we "acquire" another obstacle.

}