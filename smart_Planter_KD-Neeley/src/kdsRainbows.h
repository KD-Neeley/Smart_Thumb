/* 
 * Project: kdsRainbows.h
 * Description: Header file for color hex codes
 * Katie Neeley
 * 03-19-2023
 */

#ifndef _RAINBOWS_H_
#define _RAINBOWS_H_

const int nada = 0x000000;
const int full = 0xFFFFFF;
const int fullred = 0xFF0000;
const int fullgreen = 0x00FF00;
const int fullblue = 0x0000FF;
const int fullmagenta = 0xFF00FF;
const int fullyellow = 0xFFFF00;
const int fullcyan = 0x00FFFF;
const int orange = 0xFF7700;
const int turquoise = 0x00FF77;
const int purple = 0x7700FF;


const int primaries[] = {fullred, fullgreen, fullblue};
const int secondaries[] = {fullmagenta, fullyellow, fullcyan};
const int fullcolors[] = {fullred, fullgreen, fullblue, fullmagenta, fullyellow, fullcyan};
const int traditional[] = {fullred, orange, fullyellow, fullgreen, fullblue, purple};
const int fullrainbow[] = {fullred, fullmagenta, fullblue, fullcyan, fullgreen, fullyellow};
const int rainbow[] = {fullred, fullmagenta, orange, fullyellow, fullgreen, turquoise, fullcyan, fullblue, purple};
const int traditionalsecondaries[] = {orange, fullgreen, purple};
const int traditionalprimaries[] = {fullred, fullyellow, fullblue};
const int traditionalrainbow[] = {fullred, fullmagenta, orange, fullyellow, fullgreen, fullcyan, turquoise, fullblue, purple};


#endif // _RAINBOWS_H_