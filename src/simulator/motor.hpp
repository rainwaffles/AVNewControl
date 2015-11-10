#include <armadillo>
#include <math.h>

// I never actually use this
enum motors {SRGE_L, SRGE_R, DIAG_L, DIAG_R, VERT_FL, VERT_FR,VERT_BL, VERT_BR, STRAFE};

// Power of each motor
int motorPower[9];

// Position of each motor
arma::vec motorPos[9];
// Direction of each motor
arma::vec motorDir[9];
