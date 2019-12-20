#ifndef RECASTCONFIG_H
#define RECASTCONFIG_H

// Defines the number of bits allocated to rcSpan::smin and rcSpan::smax.
// If the mesh is appearing below the actual terrain, this parameter may need to be increased.
// rcSpan::sminおよびrcSpan::smaxに割り当てられるビット数を定義します。
//メッシュが実際の地形の下に表示される場合、このパラメーターを増やす必要がある場合があります。
constexpr int RC_SPAN_HEIGHT_BITS = 13;

#endif // RECASTCONFIG_H