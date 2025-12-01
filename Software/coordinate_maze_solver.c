#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#define MAX_STACK 100
#define MAZE_SIZE 6

/* Direction enumeration for cleaner code */
typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
} Direction;

/* Position structure */
typedef struct {
    int x;
    int y;
} Position;

/* Stack for DFS backtracking */
typedef struct {
    Position pos;
    Direction dir;
} StackNode;

StackNode stack[MAX_STACK];
int stack_top = -1;

/* Visited grid to prevent loops */
bool visited[MAZE_SIZE][MAZE_SIZE] = {false};

/* Current robot state */
Position current_pos;
Direction current_dir;

/* IR sensor readings */
bool ir_north, ir_south, ir_west, ir_east;

/* Direction names for printing */
const char* dir_names[] = {"NORTH", "EAST", "SOUTH", "WEST"};

/* Function prototypes */
void push_stack(Position pos, Direction dir);
bool pop_stack(Position *pos, Direction *dir);
Position get_next_position(Position pos, Direction dir);
void turn_robot(Direction new_dir);
bool is_valid_position(int x, int y);
Direction get_direction_from_input(void);

void push_stack(Position pos, Direction dir) {
    if (stack_top < MAX_STACK - 1) {
        stack_top++;
        stack[stack_top].pos = pos;
        stack[stack_top].dir = dir;
    }
}

bool pop_stack(Position *pos, Direction *dir) {
    if (stack_top >= 0) {
        *pos = stack[stack_top].pos;
        *dir = stack[stack_top].dir;
        stack_top--;
        return true;
    }
    return false;
}

Position get_next_position(Position pos, Direction dir) {
    Position next = pos;
    switch(dir) {
        case NORTH: next.y--; break;
        case SOUTH: next.y++; break;
        case EAST:  next.x++; break;
        case WEST:  next.x--; break;
    }
    return next;
}

bool is_valid_position(int x, int y) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

void turn_robot(Direction new_dir) {
    if (current_dir != new_dir) {
        printf("Turning from %s to %s\n", dir_names[current_dir], dir_names[new_dir]);
        current_dir = new_dir;
    }
}

Direction get_direction_from_input(void) {
    int choice;
    printf("\nSelect starting direction:\n");
    printf("0 - NORTH\n");
    printf("1 - EAST\n");
    printf("2 - SOUTH\n");
    printf("3 - WEST\n");
    printf("Enter choice (0-3): ");
    scanf("%d", &choice);

    while (choice < 0 || choice > 3) {
        printf("Invalid choice. Enter 0-3: ");
        scanf("%d", &choice);
    }

    return (Direction)choice;
}

int main() {
    int temp;
    bool goal_reached = false;
    int moves = 0;

    printf("=== DFS Maze Solver ===\n");
    printf("Maze size: %dx%d\n", MAZE_SIZE, MAZE_SIZE);
    printf("Coordinates range from (0,0) to (%d,%d)\n\n", MAZE_SIZE-1, MAZE_SIZE-1);

    /* Get initial position from user */
    printf("Enter starting X coordinate (0-%d): ", MAZE_SIZE-1);
    scanf("%d", &current_pos.x);
    while (current_pos.x < 0 || current_pos.x >= MAZE_SIZE) {
        printf("Invalid X. Enter 0-%d: ", MAZE_SIZE-1);
        scanf("%d", &current_pos.x);
    }

    printf("Enter starting Y coordinate (0-%d): ", MAZE_SIZE-1);
    scanf("%d", &current_pos.y);
    while (current_pos.y < 0 || current_pos.y >= MAZE_SIZE) {
        printf("Invalid Y. Enter 0-%d: ", MAZE_SIZE-1);
        scanf("%d", &current_pos.y);
    }

    /* Get initial direction from user */
    current_dir = get_direction_from_input();

    printf("\nStarting at position (%d, %d) facing %s\n",
           current_pos.x, current_pos.y, dir_names[current_dir]);
    printf("=====================================\n");

    /* Mark starting position as visited */
    visited[current_pos.y][current_pos.x] = true;

    while(!goal_reached && moves < 100) {  // Safety limit
        moves++;
        printf("\n--- Move %d: Position (%d, %d), Facing %s ---\n",
               moves, current_pos.x, current_pos.y, dir_names[current_dir]);

        /* Get IR sensor readings */
        printf("North scan (1=wall, 0=open): ");
        scanf("%d", &temp);
        ir_north = temp;
        printf("South scan: ");
        scanf("%d", &temp);
        ir_south = temp;
        printf("West scan: ");
        scanf("%d", &temp);
        ir_west = temp;
        printf("East scan: ");
        scanf("%d", &temp);
        ir_east = temp;

        /* Store wall info in array for easier processing */
        bool walls[4] = {ir_north, ir_east, ir_south, ir_west};
        int wall_count = ir_north + ir_south + ir_west + ir_east;

        /* Check if goal reached (you can define your own goal condition) */
        if (wall_count == 4) {
            printf("\n*** TRAPPED: All directions blocked ***\n");
            break;
        }

        /* DFS Algorithm: Try directions in priority order */
        /* Priority: Forward, Right, Left, Backward (typical DFS) */
        Direction try_order[4];
        try_order[0] = current_dir;                    // Forward
        try_order[1] = (current_dir + 1) % 4;          // Right
        try_order[2] = (current_dir + 3) % 4;          // Left
        try_order[3] = (current_dir + 2) % 4;          // Backward

        bool moved = false;

        for (int i = 0; i < 4; i++) {
            Direction try_dir = try_order[i];

            /* Check if this direction is open (no wall) */
            if (walls[try_dir]) {
                continue;  // Wall present, skip
            }

            /* Calculate next position */
            Position next_pos = get_next_position(current_pos, try_dir);

            /* Check if valid and unvisited */
            if (!is_valid_position(next_pos.x, next_pos.y)) {
                printf("  %s: Out of bounds\n", dir_names[try_dir]);
                continue;
            }

            if (visited[next_pos.y][next_pos.x]) {
                printf("  %s: Already visited\n", dir_names[try_dir]);
                continue;
            }

            /* Valid move found! */
            printf("  %s: Open and unvisited - MOVING\n", dir_names[try_dir]);

            /* Push current state to stack before moving */
            push_stack(current_pos, current_dir);

            /* Turn and move */
            turn_robot(try_dir);
            current_pos = next_pos;
            visited[next_pos.y][next_pos.x] = true;

            printf("Moved to (%d, %d)\n", current_pos.x, current_pos.y);
            moved = true;
            break;
        }

        /* If no valid moves, backtrack */
        if (!moved) {
            printf("\nDEAD END - Backtracking...\n");

            Position backtrack_pos;
            Direction backtrack_dir;

            if (pop_stack(&backtrack_pos, &backtrack_dir)) {
                printf("Backtracking to (%d, %d)\n", backtrack_pos.x, backtrack_pos.y);
                current_pos = backtrack_pos;
                current_dir = backtrack_dir;
            } else {
                printf("\n*** MAZE FULLY EXPLORED - No solution found ***\n");
                break;
            }
        }

        /* Optional: Define goal condition */
        // Example: if (current_pos.x == 5 && current_pos.y == 5) {
        //     goal_reached = true;
        //     printf("\n*** GOAL REACHED! ***\n");
        // }
    }

    printf("\n=== Maze solving complete ===\n");
    printf("Total moves: %d\n", moves);
    printf("Final position: (%d, %d)\n", current_pos.x, current_pos.y);

    return 0;
}
