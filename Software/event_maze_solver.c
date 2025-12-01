#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#define MAX_EVENTS 100
#define MAX_STACK 100

/* Direction enumeration */
typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
} Direction;

/* Event types the robot can encounter */
typedef enum {
    EVENT_JUNCTION,
    EVENT_TURN,
    EVENT_STRAIGHT,
    EVENT_DEAD_END
} EventType;

/* Structure to remember a location/event */
typedef struct {
    EventType type;
    int event_id;
    Direction arrival_dir;
    bool explored_paths[4];
    int steps_from_prev;
} MazeEvent;

/* Stack for backtracking */
typedef struct {
    int event_id;
    Direction facing;
} StackNode;

/* Global state */
MazeEvent events[MAX_EVENTS];
int event_count = 0;
int current_event_id = -1;

StackNode path_stack[MAX_STACK];
int stack_top = -1;

Direction current_dir;
int steps_since_last_event = 0;

/* IR sensor readings */
bool ir_north, ir_south, ir_west, ir_east;

/* Direction names */
const char* dir_names[] = {"NORTH", "EAST", "SOUTH", "WEST"};
const char* event_names[] = {"JUNCTION", "TURN", "STRAIGHT", "DEAD_END"};

/* Function prototypes */
void push_stack(int event_id, Direction dir);
bool pop_stack(int *event_id, Direction *dir);
Direction turn_left(Direction dir);
Direction turn_right(Direction dir);
Direction turn_around(Direction dir);
int count_open_paths(bool walls[4]);
EventType classify_location(bool walls[4], int open_count);
int find_or_create_event(EventType type, Direction arrival_dir);
Direction choose_next_direction(int event_id, bool walls[4]);

void push_stack(int event_id, Direction dir) {
    if (stack_top < MAX_STACK - 1) {
        stack_top++;
        path_stack[stack_top].event_id = event_id;
        path_stack[stack_top].facing = dir;
    }
}

bool pop_stack(int *event_id, Direction *dir) {
    if (stack_top >= 0) {
        *event_id = path_stack[stack_top].event_id;
        *dir = path_stack[stack_top].facing;
        stack_top--;
        return true;
    }
    return false;
}

Direction turn_left(Direction dir) {
    return (dir + 3) % 4;
}

Direction turn_right(Direction dir) {
    return (dir + 1) % 4;
}

Direction turn_around(Direction dir) {
    return (dir + 2) % 4;
}

int count_open_paths(bool walls[4]) {
    int count = 0;
    for (int i = 0; i < 4; i++) {
        if (!walls[i]) count++;
    }
    return count;
}

EventType classify_location(bool walls[4], int open_count) {
    if (open_count == 1) {
        return EVENT_DEAD_END;
    } else if (open_count == 2) {
        if ((walls[NORTH] && walls[SOUTH]) || (walls[EAST] && walls[WEST])) {
            return EVENT_STRAIGHT;
        } else {
            return EVENT_TURN;
        }
    } else {
        return EVENT_JUNCTION;
    }
}

int find_or_create_event(EventType type, Direction arrival_dir) {
    if (type == EVENT_STRAIGHT) {
        return -1;
    }

    int id = event_count;
    events[id].type = type;
    events[id].event_id = id;
    events[id].arrival_dir = arrival_dir;
    events[id].steps_from_prev = steps_since_last_event;

    for (int i = 0; i < 4; i++) {
        events[id].explored_paths[i] = false;
    }

    // Mark the direction we arrived from as already explored
    Direction came_from = turn_around(arrival_dir);
    events[id].explored_paths[came_from] = true;

    event_count++;
    steps_since_last_event = 0;

    return id;
}

Direction choose_next_direction(int event_id, bool walls[4]) {
    if (event_id < 0) return current_dir;

    MazeEvent *event = &events[event_id];

    // Priority: Forward, Right, Left, Backward
    Direction try_order[4];
    try_order[0] = current_dir;
    try_order[1] = turn_right(current_dir);
    try_order[2] = turn_left(current_dir);
    try_order[3] = turn_around(current_dir);

    for (int i = 0; i < 4; i++) {
        Direction try_dir = try_order[i];

        if (event->explored_paths[try_dir]) {
            printf("  %s: Already explored from this location\n", dir_names[try_dir]);
            continue;
        }

        if (walls[try_dir]) {
            event->explored_paths[try_dir] = true;
            continue;
        }

        event->explored_paths[try_dir] = true;
        return try_dir;
    }

    return -1;
}

int main() {
    int temp;
    int moves = 0;

    printf("=== Event-Based DFS Maze Solver ===\n");
    printf("Robot uses only IR sensors and remembers locations as events\n\n");

    int choice;
    printf("Select starting direction:\n");
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

    current_dir = (Direction)choice;
    printf("\nStarting, facing %s\n", dir_names[current_dir]);
    printf("=====================================\n");

    while(moves < 200) {
        moves++;
        printf("\n--- Move %d: Facing %s", moves, dir_names[current_dir]);
        if (current_event_id >= 0) {
            printf(" (at Event #%d: %s)", current_event_id, event_names[events[current_event_id].type]);
        }
        printf(" ---\n");

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

        bool walls[4] = {ir_north, ir_east, ir_south, ir_west};
        int open_count = count_open_paths(walls);

        printf("Sensed: %d open paths\n", open_count);

        if (open_count == 0) {
            printf("\n*** TRAPPED: All directions blocked ***\n");
            break;
        }

        EventType location_type = classify_location(walls, open_count);

        if (location_type == EVENT_STRAIGHT) {
            printf("Straight corridor - continuing %s\n", dir_names[current_dir]);
            steps_since_last_event++;
            current_event_id = -1;

        } else if (location_type == EVENT_DEAD_END) {
            printf("Location type: DEAD_END\n");

            if (current_event_id == -1) {
                current_event_id = find_or_create_event(location_type, current_dir);
                printf("Created Event #%d (dead end)\n", current_event_id);
            }

            printf("DEAD END - Must backtrack\n");

            int prev_event;
            Direction prev_dir;

            if (pop_stack(&prev_event, &prev_dir)) {
                printf("Backtracking to Event #%d\n", prev_event);

                // Mark the direction we WENT (not where we're returning from) as explored
                // prev_dir is the direction we left the previous event
                // So that's the direction that's now explored
                events[prev_event].explored_paths[prev_dir] = true;
                printf("Marking %s as explored at Event #%d (the direction we just explored)\n",
                       dir_names[prev_dir], prev_event);

                current_event_id = prev_event;
                current_dir = turn_around(prev_dir);  // Face back toward the event
                steps_since_last_event = 0;
            } else {
                printf("\n*** MAZE FULLY EXPLORED ***\n");
                break;
            }

        } else {
            printf("Location type: %s\n", event_names[location_type]);

            if (current_event_id == -1) {
                current_event_id = find_or_create_event(location_type, current_dir);
                printf("Created Event #%d (%d steps from previous event)\n",
                       current_event_id, events[current_event_id].steps_from_prev);
                printf("Marked %s as already explored (direction we came from)\n",
                       dir_names[turn_around(current_dir)]);
            } else {
                printf("At Event #%d - choosing new path\n", current_event_id);
            }

            Direction next_dir = choose_next_direction(current_event_id, walls);

            if (next_dir == -1) {
                printf("All paths explored from this event - Backtracking\n");

                int prev_event;
                Direction prev_dir;

                if (pop_stack(&prev_event, &prev_dir)) {
                    printf("Backtracking to Event #%d\n", prev_event);

                    // Mark the direction we WENT as explored
                    events[prev_event].explored_paths[prev_dir] = true;
                    printf("Marking %s as explored at Event #%d (the direction we just explored)\n",
                           dir_names[prev_dir], prev_event);

                    current_event_id = prev_event;
                    current_dir = turn_around(prev_dir);
                    steps_since_last_event = 0;
                } else {
                    printf("\n*** MAZE FULLY EXPLORED ***\n");
                    break;
                }
            } else {
                printf("Choosing to go %s\n", dir_names[next_dir]);

                if (next_dir != current_dir) {
                    printf("Turning from %s to %s\n", dir_names[current_dir], dir_names[next_dir]);
                    current_dir = next_dir;
                }

                push_stack(current_event_id, next_dir);
                steps_since_last_event = 1;
                current_event_id = -1;
            }
        }
    }

    printf("\n=== Maze solving complete ===\n");
    printf("Total moves: %d\n", moves);
    printf("Total events discovered: %d\n", event_count);
    printf("\nEvent summary:\n");
    for (int i = 0; i < event_count; i++) {
        printf("Event #%d: %s (arrived facing %s, %d steps from previous)\n",
               i, event_names[events[i].type], dir_names[events[i].arrival_dir],
               events[i].steps_from_prev);
    }

    return 0;
}
