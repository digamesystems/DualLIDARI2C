#ifndef DIGAME_DEBUG_H_INCLUDED
#define DIGAME_DEBUG_H_INCLUDED

// Show debug messages.
#define SHOW_DEBUG

// Define where debug output will be printed.
#define DEBUG_PRINTER Serial

// Setup debug printing macros.
#ifdef SHOW_DEBUG
  #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) {}
  #define DEBUG_PRINTLN(...) {}
#endif

#endif  // DIGAME_DEBUG_H_INCLUDED