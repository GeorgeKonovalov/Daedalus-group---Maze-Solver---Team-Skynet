/*
  MazeTypes.h — Type definitions for Event-Based DFS Maze Solver
  ===============================================================
  The maze is modelled as a graph of events (junctions, turns, dead ends).
  Corridors are not stored — they are just the paths between events.
  No grid, no coordinates, no maze size assumptions.
*/

#ifndef MAZE_TYPES_H
#define MAZE_TYPES_H

#include <Arduino.h>

// =====================================================================
//  RUN MODE
// =====================================================================
enum RunMode : uint8_t { SENSOR_TEST, CORRIDOR_PID, MAZE_DFS };

// =====================================================================
//  DIRECTIONS  (absolute — robot body never rotates)
// =====================================================================
enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// =====================================================================
//  EVENT TYPE — classified relative to arrival heading
// =====================================================================
enum EventType : uint8_t {
    EVT_DEAD_END,        // walls front + left + right
    EVT_T_JUNCTION,      // wall front, open left + right
    EVT_LEFT_TURN,       // wall front + right, open left
    EVT_RIGHT_TURN,      // wall front + left, open right
    EVT_LEFT_BRANCH,     // open front + left, wall right
    EVT_RIGHT_BRANCH,    // open front + right, wall left
    EVT_START            // starting position (no arrival corridor)
};

// =====================================================================
//  EVENT — one node in the maze graph
// =====================================================================
//  Each event records:
//    exits[]       — which absolute directions are physically open
//    explored[]    — which of those exits DFS has already tried
//    entryHeading  — heading when the robot first arrived here
//                    (used to compute the backtrack direction)
//
//  Corridors between events are implicit.  The robot PID-follows
//  through them without saving anything.  To backtrack, it simply
//  reverses its heading — no coordinates needed.
// =====================================================================
struct Event {
    EventType type;
    bool      exits[4];       // true = open in that absolute direction
    bool      explored[4];    // true = DFS already went that way
    Direction entryHeading;   // heading at first arrival
};

// =====================================================================
//  STATE MACHINE
// =====================================================================
enum RobotState : uint8_t {
    ST_STARTUP,
    ST_SCAN_EVENT,    // stopped at event, reading sensors
    ST_DECIDE,        // DFS picking next direction
    ST_FOLLOWING,     // PID corridor following to next event
    ST_COMPLETE       // maze fully explored
};

#endif
