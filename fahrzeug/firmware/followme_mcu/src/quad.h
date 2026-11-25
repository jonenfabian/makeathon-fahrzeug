// quad.h — Auswertung eines Quadratur-Encoders (Hall-Encoder am Motor).
//
// Quadratur-Encoder: zwei Signale A und B, um eine Viertelperiode
// versetzt. Aus der Reihenfolge der Flanken folgt die Drehrichtung:
//   vorwärts  00 → 01 → 11 → 10 → 00   (Bit 1 = A, Bit 0 = B)
//   rückwärts 00 → 10 → 11 → 01 → 00
// Wertet man jede Flanke beider Kanäle aus ("Vierfachauswertung"), ergibt
// das 4 Zählschritte pro Encoder-Impuls. Übergänge, bei denen sich beide
// Bits gleichzeitig ändern (z. B. 00 → 11), sind ungültig (verpasste
// Flanke) und zählen 0.
#pragma once
#include <stdint.h>

namespace quad {

// Index = (alter Zustand << 2) | neuer Zustand.
constexpr int8_t kTable[16] = {
    0,  +1, -1, 0,   // alt 00 → neu 00, 01, 10, 11
    -1, 0,  0,  +1,  // alt 01 → 00, 01, 10, 11
    +1, 0,  0,  -1,  // alt 10 → 00, 01, 10, 11
    0,  -1, +1, 0,   // alt 11 → 00, 01, 10, 11
};

// Liefert den Zählschritt für den Übergang prev → curr (je 2 Bit A|B).
inline int8_t step(uint8_t prev, uint8_t curr) {
  return kTable[((prev & 0x3u) << 2) | (curr & 0x3u)];
}

}  // namespace quad
