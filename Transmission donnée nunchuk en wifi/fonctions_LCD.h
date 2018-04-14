#include "GLCD.h"


void pixel_c(int x0, int y0, short couleur);

void ligne(int x0, int y0, int x1, int y1);
void ligne_c(int x0, int y0, int x1, int y1, short couleur);

void ligne_horiz(int x0, int x1, int y);
void ligne_verti(int x, int y0, int y1);

void cercle_vide(int x0, int y0, int rayon);
void cercle_vide_c(int x0, int y0, int rayon, short couleur);

void cercle_plein(int x0, int y0, int rayon);
void cercle_plein_c(int x0, int y0, int rayon, short couleur);

void ellipse_vide(int x0, int y0, int rayon_x, int rayon_y);
void ellipse_vide_c(int x0, int y0, int rayon_x, int rayon_y, short couleur);

void ellipse_plein(int x0, int y0, int rayon_x, int rayon_y);
void ellipse_plein_c(int x0, int y0, int rayon_x, int rayon_y, short couleur);

void rectangle_vide(int x0,int y0, int cote_x, int cote_y);
void rectangle_vide_c(int x0,int y0, int cote_x, int cote_y, short couleur);

void rectangle_plein(int x0,int y0, int cote_x, int cote_y);
void rectangle_plein_c(int x0,int y0, int cote_x, int cote_y, short couleur);
