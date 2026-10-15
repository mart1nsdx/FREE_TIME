/*
physics_UAV.h

Motor de fisico  del UAV — separado de physics_UAV.cpp (demo de consola)
y de drone_model.cpp (motor de simulacion / visualizacion OpenGL) para que
ambos consuman exactamente las mismas ecuaciones.

main source:
Ecuaciones de movimiento de un UAV — Beard & McLain Cap. 3

  BLOQUE 0 — Geometria de masa / inercia : forma -> masa, CG, tensor de inercia
  BLOQUE 1 — Translational Kinematics    : body velocity -> inertial position rates
  BLOQUE 2 — Translational Dynamics      : forces -> body velocity rates
  BLOQUE 3 — Rotational Kinematics       : body rates -> Euler angle rates
  BLOQUE 4 — Rotational Dynamics         : torques -> body angular rate derivatives

Ejes cuerpo: X adelante (nariz), Y ala derecha, Z abajo (NED-body).
*/

#ifndef PHYSICS_UAV_H
#define PHYSICS_UAV_H

#include <iostream>
#include <cmath>
#include <algorithm>

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
};

// Complete state of UAV
struct UAVState {
    double pn, pe, pd;       // inertial position NED (m)
    double u,  v,  w;        // body-frame velocity (m/s)
    double phi, theta, psi;  // Euler angles: roll, pitch, yaw (rad)
    double p,  q,  r;        // body-frame angular rates (rad/s)
};

// Inertia tensor (kg·m²) — Jxz is the cross-product term
struct Inertia {
    double Jx, Jy, Jz, Jxz;
};


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 0 — Geometria de masa / inercia
//
// Construye masa total, CG y tensor de inercia de un dron generico ala delta
// a partir de sus componentes geometricos (fuselaje + ala + compartimento +
// motor). Esto alimenta el Bloque 4 (dinamica rotacional): la simulacion no
// usa valores de inercia inventados, los deriva de una geometria real.
// ─────────────────────────────────────────────────────────────────────────────

// Tensor de inercia completo (para construir/verificar la geometria antes de
// reducirlo a la forma {Jx,Jy,Jz,Jxz} que consumen las ecuaciones de movimiento)
struct Inertia3x3 {
    double Ixx, Iyy, Izz;
    double Ixy, Ixz, Iyz;
};

struct MassComponent {
    const char* nombre;
    double masa;         // kg
    Vec3   cg;            // posicion del CG del componente respecto al origen [m]
    Inertia3x3 I_local;   // inercia del componente respecto a SU PROPIO cg [kg*m^2]
};

// --- Inercia de un cilindro solido (fuselaje), eje a lo largo de X ---
inline Inertia3x3 inerciaCilindro(double masa, double radio, double longitud)
{
    double Ixx = 0.5 * masa * radio * radio;
    double Iyy = (1.0/12.0) * masa * (3*radio*radio + longitud*longitud);
    double Izz = Iyy;
    return {Ixx, Iyy, Izz, 0.0, 0.0, 0.0};
}

// --- Inercia de una placa triangular delta plana en el plano XY ---
// Triangulo isosceles: base = envergadura (Y), altura = cuerda de raiz (X)
inline Inertia3x3 inerciaAlaDelta(double masa, double envergadura, double cuerdaRaiz)
{
    double b = envergadura;
    double c = cuerdaRaiz;
    double Ixx = masa * (b*b) / 24.0;   // rotacion sobre eje X (envergadura)
    double Iyy = masa * (c*c) / 18.0;   // rotacion sobre eje Y (cuerda)
    double Izz = Ixx + Iyy;             // ejes perpendiculares (placa plana)
    return {Ixx, Iyy, Izz, 0.0, 0.0, 0.0};
}

// --- Inercia de una caja rectangular (compartimento delantero) ---
inline Inertia3x3 inerciaCaja(double masa, double lx, double ly, double lz)
{
    double Ixx = (1.0/12.0) * masa * (ly*ly + lz*lz);
    double Iyy = (1.0/12.0) * masa * (lx*lx + lz*lz);
    double Izz = (1.0/12.0) * masa * (lx*lx + ly*ly);
    return {Ixx, Iyy, Izz, 0.0, 0.0, 0.0};
}

// --- Masa puntual (motor trasero) ---
inline Inertia3x3 inerciaPuntual()
{
    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

// --- Inercia de un disco delgado en el plano XY, eje de giro a lo largo de Z
// (helice de un multirrotor: gira sobre el eje vertical del cuerpo) ---
inline Inertia3x3 inerciaDiscoZ(double masa, double radio)
{
    double Izz = 0.5  * masa * radio * radio;   // eje de giro
    double Ixx = 0.25 * masa * radio * radio;   // diametral
    return {Ixx, Ixx, Izz, 0.0, 0.0, 0.0};
}

// Rota un tensor de inercia un angulo sobre el eje Z: I' = R * I * R^T.
// Hace falta para los brazos de un multirrotor en X: cada brazo es una barra
// cuya inercia se calcula facil sobre SU propio eje (inerciaCilindro, eje X),
// pero que en el cuerpo esta a 45/135/225/315 grados, asi que hay que llevar
// ese tensor a los ejes del cuerpo antes de sumarlo.
inline Inertia3x3 rotarZ(const Inertia3x3& I, double angulo)
{
    const double c = std::cos(angulo), s_ = std::sin(angulo);
    const double m[3][3] = { { I.Ixx, I.Ixy, I.Ixz },
                             { I.Ixy, I.Iyy, I.Iyz },
                             { I.Ixz, I.Iyz, I.Izz } };
    const double R[3][3] = { { c, -s_, 0.0 },
                             { s_,  c,  0.0 },
                             { 0.0, 0.0, 1.0 } };
    double out[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double acc = 0.0;
            for (int k = 0; k < 3; ++k)
                for (int l = 0; l < 3; ++l)
                    acc += R[i][k] * m[k][l] * R[j][l];
            out[i][j] = acc;
        }
    return { out[0][0], out[1][1], out[2][2], out[0][1], out[0][2], out[1][2] };
}

// Rota un tensor de inercia un angulo sobre el eje Y: I' = R * I * R^T.
// Es el equivalente de rotarZ() para el eje de cabeceo, y hace falta para las
// gondolas BASCULANTES del convertiplano: cada gondola (y su rotor) gira
// alrededor del eje y del cuerpo al pasar de sustentacion a propulsion, asi
// que su tensor de inercia —calculado facil sobre su propio eje— hay que
// llevarlo a los ejes del cuerpo con el angulo de basculacion actual.
inline Inertia3x3 rotarY(const Inertia3x3& I, double angulo)
{
    const double c = std::cos(angulo), s_ = std::sin(angulo);
    const double m[3][3] = { { I.Ixx, I.Ixy, I.Ixz },
                             { I.Ixy, I.Iyy, I.Iyz },
                             { I.Ixz, I.Iyz, I.Izz } };
    const double R[3][3] = { {  c,  0.0, s_  },
                             { 0.0, 1.0, 0.0 },
                             { -s_, 0.0, c   } };
    double out[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double acc = 0.0;
            for (int k = 0; k < 3; ++k)
                for (int l = 0; l < 3; ++l)
                    acc += R[i][k] * m[k][l] * R[j][l];
            out[i][j] = acc;
        }
    return { out[0][0], out[1][1], out[2][2], out[0][1], out[0][2], out[1][2] };
}

// Rota un VECTOR sobre el eje Y (misma R que rotarY). Se usa para llevar los
// offsets geometricos de la gondola (donde queda el rotor respecto del pivote)
// al angulo de basculacion actual: al bascular, el CG de cada gondola se
// mueve de verdad, y con el, el CG y la inercia de todo el vehiculo.
inline Vec3 rotarYVec(const Vec3& v, double angulo)
{
    const double c = std::cos(angulo), s_ = std::sin(angulo);
    return Vec3( c*v.x + s_*v.z,
                 v.y,
                -s_*v.x + c*v.z );
}

// Teorema de ejes paralelos: traslada la inercia local al origen comun
inline Inertia3x3 trasladar(const Inertia3x3& I, double masa, const Vec3& r)
{
    Inertia3x3 out;
    out.Ixx = I.Ixx + masa * (r.y*r.y + r.z*r.z);
    out.Iyy = I.Iyy + masa * (r.x*r.x + r.z*r.z);
    out.Izz = I.Izz + masa * (r.x*r.x + r.y*r.y);
    out.Ixy = I.Ixy - masa * (r.x*r.y);
    out.Ixz = I.Ixz - masa * (r.x*r.z);
    out.Iyz = I.Iyz - masa * (r.y*r.z);
    return out;
}

// Propiedades de masa listas para alimentar la simulacion
struct DroneMassProperties {
    double mass;        // kg
    Vec3   cg;           // respecto al origen geometrico del fuselaje [m]
    Inertia J;           // forma reducida {Jx,Jy,Jz,Jxz} para el Bloque 4
    Inertia3x3 I_full;   // tensor completo respecto al CG, para reporte/verificacion
};

// Acumula una lista de componentes en masa total, CG y tensor de inercia
// respecto a ese CG (teorema de ejes paralelos). Es el cierre del Bloque 0 y
// lo comparten todos los vehiculos: cada uno solo describe su geometria.
inline DroneMassProperties combinarComponentes(const MassComponent* comps, int n,
                                               const char* vehiculo)
{
    // Masa total y CG
    double masaTotal = 0.0;
    Vec3 cgAcumulado(0, 0, 0);
    for (int i = 0; i < n; ++i) {
        masaTotal += comps[i].masa;
        cgAcumulado.x += comps[i].masa * comps[i].cg.x;
        cgAcumulado.y += comps[i].masa * comps[i].cg.y;
        cgAcumulado.z += comps[i].masa * comps[i].cg.z;
    }
    Vec3 cg(cgAcumulado.x/masaTotal, cgAcumulado.y/masaTotal, cgAcumulado.z/masaTotal);

    // Tensor de inercia total respecto al CG real (teorema de ejes paralelos)
    Inertia3x3 Itot = {0,0,0,0,0,0};
    for (int i = 0; i < n; ++i) {
        Vec3 r(comps[i].cg.x - cg.x, comps[i].cg.y - cg.y, comps[i].cg.z - cg.z);
        Inertia3x3 Ic = trasladar(comps[i].I_local, comps[i].masa, r);
        Itot.Ixx += Ic.Ixx; Itot.Iyy += Ic.Iyy; Itot.Izz += Ic.Izz;
        Itot.Ixy += Ic.Ixy; Itot.Ixz += Ic.Ixz; Itot.Iyz += Ic.Iyz;
    }

    // Las ecuaciones de movimiento (Bloque 4) asumen simetria de masa respecto
    // al plano XZ (Ixy = Iyz = 0). Tanto el ala delta (todo en Y=0) como el
    // quadcopter (cuatro ramas simetricas) lo cumplen por construccion; se
    // avisa si alguna vez deja de serlo.
    if (std::abs(Itot.Ixy) > 1e-6 || std::abs(Itot.Iyz) > 1e-6) {
        std::cerr << "[WARNING] La geometria de " << vehiculo << " no es simetrica "
                     "respecto al plano XZ (Ixy=" << Itot.Ixy << ", Iyz=" << Itot.Iyz
                  << "); rotational_dynamics() los ignora.\n";
    }

    Inertia J{ Itot.Ixx, Itot.Iyy, Itot.Izz, Itot.Ixz };

    return { masaTotal, cg, J, Itot };
}

// Geometria de referencia: dron generico ala delta
//   fuselaje central (cilindro) + ala delta (placa triangular)
//   + compartimento delantero (caja) + motor trasero (masa puntual)
inline DroneMassProperties buildDeltaWingDrone()
{
    const double envergadura   = 2.5;   // m
    const double longitudTotal = 3.5;   // m
    const double cuerdaRaiz    = 0.95;  // m
    const double radioFuselaje = 0.16;  // m

    MassComponent comps[4];

    // 1) Fuselaje (cilindro), centrado en el origen
    comps[0] = { "Fuselaje",
                 90.0,
                 {0.0, 0.0, 0.0},
                 inerciaCilindro(90.0, radioFuselaje, longitudTotal) };

    // 2) Ala delta: vertice delantero a x_ala, centroide a 1/3 de la cuerda de raiz detras
    double x_ala = 0.30;
    comps[1] = { "Ala delta",
                 55.0,
                 {x_ala - cuerdaRaiz/3.0, 0.0, 0.0},
                 inerciaAlaDelta(55.0, envergadura, cuerdaRaiz) };

    // 3) Compartimento delantero (caja), en la nariz
    comps[2] = { "Compartimento delantero",
                 45.0,
                 {1.30, 0.0, 0.0},
                 inerciaCaja(45.0, 0.5, 0.28, 0.28) };

    // 4) Motor + helice (masa puntual), en la cola
    comps[3] = { "Motor trasero",
                 10.0,
                 {-1.55, 0.0, 0.0},
                 inerciaPuntual() };

    return combinarComponentes(comps, 4, "ala delta");
}


// Geometria de referencia: quadcopter en configuracion X
//   cuerpo central (caja) + 4 brazos (barras a 45/135/225/315 grados)
//   + 4 motores (masas puntuales) + 4 rotores (discos) + tren de aterrizaje
//
// Marco cuerpo NED: x adelante, y a la derecha, z HACIA ABAJO — por eso los
// motores y rotores, que van arriba del cuerpo, quedan en z negativa, y el
// tren de aterrizaje en z positiva. Es la misma maquinaria del ala delta
// (componentes -> ejes paralelos -> tensor), solo cambia la geometria: la
// simulacion sigue sin usar inercias inventadas.
inline DroneMassProperties buildQuadcopter()
{
    const double brazoLargo  = 1.60;   // m — del centro a la punta del brazo
    const double brazoRadio  = 0.05;   // m
    const double rotorRadio  = 0.55;   // m

    MassComponent comps[14];
    int k = 0;

    // 1) Cuerpo central (caja): bateria + aviónica, centrado en el origen
    comps[k++] = { "Cuerpo central",
                   70.0,
                   {0.0, 0.0, 0.0},
                   inerciaCaja(70.0, 0.90, 0.90, 0.36) };

    // 2) Tren de aterrizaje (caja baja, debajo del cuerpo -> z positiva)
    comps[k++] = { "Tren de aterrizaje",
                   6.0,
                   {0.0, 0.0, 0.32},
                   inerciaCaja(6.0, 1.10, 0.90, 0.50) };

    // 3) Brazos, motores y rotores — 4 ramas a 45 grados del eje x (config X).
    //    El brazo es una barra sobre SU eje: se calcula con inerciaCilindro
    //    (eje X) y se rota a su angulo con rotarZ antes de acumularlo.
    const double angulos[4] = { M_PI/4.0, 3.0*M_PI/4.0, 5.0*M_PI/4.0, 7.0*M_PI/4.0 };
    for (int i = 0; i < 4; ++i) {
        const double a  = angulos[i];
        const double cx = std::cos(a), cy = std::sin(a);

        comps[k++] = { "Brazo",
                       6.0,
                       { brazoLargo/2.0 * cx, brazoLargo/2.0 * cy, 0.0 },
                       rotarZ(inerciaCilindro(6.0, brazoRadio, brazoLargo), a) };

        comps[k++] = { "Motor",
                       8.0,
                       { brazoLargo * cx, brazoLargo * cy, -0.10 },
                       inerciaPuntual() };

        comps[k++] = { "Rotor",
                       2.5,
                       { brazoLargo * cx, brazoLargo * cy, -0.20 },
                       inerciaDiscoZ(2.5, rotorRadio) };
    }

    return combinarComponentes(comps, k, "quadcopter");
}



// ─────────────────────────────────────────────────────────────────────────────
// Geometria de referencia: CONVERTIPLANO DE 4 ROTORES BASCULANTES (tiltrotor)
//
//   fuselaje (cilindro) + ala trapezoidal en flecha (2 paneles) + deriva
//   (placa vertical) + estabilizador horizontal + 4 gondolas basculantes
//   (cilindros) + 4 rotores (discos) + bahia de carga + tren de aterrizaje
//
// Es el tercer vehiculo del simulador y el unico HIBRIDO: despega como
// multirrotor (gondolas verticales, empuje por -Z cuerpo) y vuela como avion
// (gondolas horizontales, empuje por +X cuerpo y sustentacion del ala). El
// angulo de basculacion sigma va de 0 (vuelo estacionario) a pi/2 (crucero).
//
// Lo que lo hace distinto en el Bloque 0: su geometria de masa NO es fija.
// Al bascular, las 4 gondolas y sus rotores rotan alrededor del eje y del
// cuerpo, asi que tanto el CG como el tensor de inercia son funcion de sigma
// — buildTiltrotor(sigma) los recalcula. La simetria respecto al plano XZ se
// conserva siempre (las 4 ramas van de a pares en +-y), asi que el Bloque 4
// reducido sigue siendo valido.
//
// Dimensiones REALES en metros: el vehiculo es intermedio entre los otros dos
// (4.20 m de largo y 5.60 m de envergadura, 155 kg, contra 3.5 m/200 kg del
// ala delta y 3.2 m/142 kg del quad). Estas mismas cifras son las que usa la
// visualizacion 3D para dibujarlo a escala.
// ─────────────────────────────────────────────────────────────────────────────
struct TiltrotorGeom {
    // Fuselaje (cilindro sobre el eje x)
    double fusLargo, fusRadio;

    // Ala trapezoidal en flecha (2 paneles simetricos)
    double envergadura;      // b [m], punta a punta
    double cuerdaRaiz;       // c_r [m]
    double cuerdaPunta;      // c_t [m]
    double xAla;             // x del 1/4 de cuerda en la raiz [m]
    double barrido;          // cuanto retrasa la punta respecto de la raiz [m]
    double diedro;           // cuanto sube la punta (NED: z negativo) [m]

    // Deriva vertical (placa en el plano XZ) y estabilizador horizontal
    double derivaAltura, derivaCuerda, xDeriva;
    double estabEnverg,  estabCuerda,  xEstab;

    // Gondolas basculantes: par delantero (interior) y par trasero (exterior)
    double xGondFwd, yGondFwd;   // pivote del par delantero [m]
    double xGondAft, yGondAft;   // pivote del par trasero  [m]
    double zGond;                // altura del eje de basculacion (NED: <0 = arriba)
    double gondLargo, gondRadio;
    double rotorRadio;
    double rotorOffset;          // rotor por delante del pivote, en modo avion [m]

    // Bahia de carga (caja) y tren de aterrizaje
    double xCarga, cargaLx, cargaLy, cargaLz;
    double trenLx, trenLy, trenLz, zTren;

    // Masas [kg] — suman 155 kg
    double mFus, mAlaPanel, mDeriva, mEstab, mGond, mRotor, mCarga, mTren;
};

inline TiltrotorGeom tiltrotorGeom()
{
    TiltrotorGeom g;
    g.fusLargo    = 4.20;  g.fusRadio    = 0.22;

    g.envergadura = 5.60;  g.cuerdaRaiz  = 1.20;  g.cuerdaPunta = 0.62;
    g.xAla        = 0.05;  g.barrido     = 0.55;  g.diedro      = 0.12;

    g.derivaAltura = 1.25; g.derivaCuerda = 1.00; g.xDeriva = -1.65;
    g.estabEnverg  = 1.70; g.estabCuerda  = 0.55; g.xEstab  = -1.80;

    g.xGondFwd = 1.05;  g.yGondFwd = 1.10;
    g.xGondAft = -1.05; g.yGondAft = 2.05;
    g.zGond    = -0.18;
    g.gondLargo = 0.90; g.gondRadio = 0.20;
    g.rotorRadio = 0.58; g.rotorOffset = 0.45;

    g.xCarga = 0.85; g.cargaLx = 0.80; g.cargaLy = 0.34; g.cargaLz = 0.34;
    g.trenLx = 1.30; g.trenLy = 1.50; g.trenLz = 0.45;  g.zTren = 0.42;

    g.mFus   = 48.0; g.mAlaPanel = 15.0; g.mDeriva = 5.0; g.mEstab = 3.5;
    g.mGond  =  8.0; g.mRotor    =  2.6; g.mCarga  = 21.0; g.mTren  = 5.0;
    return g;
}

// Superficie alar (trapecio) y cuerda media — alimentan el modelo
// aerodinamico del Bloque 2b, para que el ala del dibujo, la del tensor de
// inercia y la de la sustentacion sean literalmente la misma ala.
inline double tiltrotorAreaAlar(const TiltrotorGeom& g)
{
    return g.envergadura * 0.5 * (g.cuerdaRaiz + g.cuerdaPunta);
}
inline double tiltrotorCuerdaMedia(const TiltrotorGeom& g)
{
    return 0.5 * (g.cuerdaRaiz + g.cuerdaPunta);
}

// Direccion del empuje de una gondola en el marco cuerpo, para un angulo de
// basculacion sigma: sigma=0 -> (0,0,-1) (hacia arriba, vuelo estacionario);
// sigma=pi/2 -> (1,0,0) (hacia adelante, crucero). Es el vector que usan
// tanto la vectorizacion del empuje (Bloque 2) como la orientacion de las
// gondolas en el dibujo 3D.
inline Vec3 tiltThrustAxis(double sigma)
{
    return Vec3(std::sin(sigma), 0.0, -std::cos(sigma));
}

// ─────────────────────────────────────────────────────────────────────────────
// Modelo de helice — caida de empuje con la velocidad de vuelo
//
// Un rotor NO da el mismo empuje parado que avanzando: a medida que crece la
// velocidad del aire que entra por el disco, la helice puede acelerarlo cada
// vez menos, y el empuje cae. Es la teoria de cantidad de movimiento
// (Beard & McLain eq. 4.18):
//
//     T = 1/2 * rho * S_prop * C_prop * ( (k*delta_t)^2 - Va_ax^2 )
//
// Escrito con el empuje estatico lineal en la palanca (para que la palanca
// signifique lo mismo que en los otros dos vehiculos) queda:
//
//     T(delta, Va_ax) = delta * T_estatico  -  kLapse * Va_ax^2
//
// donde Va_ax es la componente de la velocidad aerodinamica SOBRE EL EJE DE
// LA GONDOLA (la unica que atraviesa el disco): Va_ax = u*sin(s) - w*cos(s).
//
// Para el convertiplano esto no es un detalle: el empuje esta dimensionado
// para sostener el peso en vuelo estacionario (T/W = 2), que es muchisimo mas
// del que hace falta para vencer el arrastre en crucero. Sin la caida, en
// modo avion el vehiculo acelera sin techo util. Con ella, el empuje
// disponible se desploma justo cuando el ala ya sostiene el peso — que es
// exactamente el compromiso que define a un convertiplano real.
//
// Solo se cuenta el flujo que ENTRA por delante del disco (Va_ax > 0): en
// descenso vertical el rotor trabaja en otro regimen, que este modelo simple
// no pretende cubrir.
// ─────────────────────────────────────────────────────────────────────────────
struct PropModel {
    double thrustStatic;   // N — empuje con palanca 1.0 y velocidad nula
    double kLapse;         // N/(m/s)^2 — 1/2 * rho * S_prop * C_prop
};

inline PropModel tiltPropModel(const TiltrotorGeom& g, double thrustStatic,
                               double rho = 1.2682, double Cprop = 1.0)
{
    const double Sprop = 4.0 * M_PI * g.rotorRadio * g.rotorRadio;   // 4 discos
    return { thrustStatic, 0.5 * rho * Sprop * Cprop };
}

// Velocidad aerodinamica proyectada sobre el eje de empuje de las gondolas
inline double tiltAxialAirspeed(const UAVState& s, double sigma)
{
    const Vec3 n = tiltThrustAxis(sigma);
    return s.u * n.x + s.w * n.z;
}

inline double tiltThrust(const PropModel& pr, double throttle,
                         const UAVState& s, double sigma)
{
    const double vax = std::max(0.0, tiltAxialAirspeed(s, sigma));
    return std::max(0.0, throttle * pr.thrustStatic - pr.kLapse * vax * vax);
}

// Posicion del cubo del rotor i (0=del-der, 1=del-izq, 2=tras-der, 3=tras-izq)
// en el marco cuerpo, para la basculacion sigma. Sale de rotar el offset del
// pivote, asi que el rotor "barre" el mismo arco que en el vehiculo real.
inline Vec3 tiltRotorHub(const TiltrotorGeom& g, int i, double sigma)
{
    const bool  del = (i < 2);
    const double xp = del ? g.xGondFwd : g.xGondAft;
    const double yp = del ? g.yGondFwd : g.yGondAft;
    const double sy = (i % 2 == 0) ? +1.0 : -1.0;   // par: derecha, impar: izquierda

    // tau = pi/2 - sigma lleva el offset de crucero (+x) al de estacionario (-z)
    const Vec3 off = rotarYVec(Vec3(g.rotorOffset, 0.0, 0.0), M_PI/2.0 - sigma);
    return Vec3(xp + off.x, sy * yp, g.zGond + off.z);
}

// Bloque 0 del convertiplano — masa, CG y tensor de inercia para la
// basculacion sigma [rad]. Misma maquinaria que los otros dos vehiculos
// (componentes -> ejes paralelos -> tensor), con una sola diferencia: las
// gondolas y los rotores entran rotados por sigma, asi que el resultado
// depende del modo de vuelo.
inline DroneMassProperties buildTiltrotor(double sigma)
{
    const TiltrotorGeom g = tiltrotorGeom();
    const double tau = M_PI/2.0 - sigma;   // angulo que lleva el eje +x al eje de la gondola

    MassComponent comps[16];
    int k = 0;

    // 1) Fuselaje (cilindro sobre x), centrado en el origen geometrico
    comps[k++] = { "Fuselaje",
                   g.mFus,
                   {0.0, 0.0, 0.0},
                   inerciaCilindro(g.mFus, g.fusRadio, g.fusLargo) };

    // 2) Bahia de carga / avionica (caja) adelante
    comps[k++] = { "Bahia de carga",
                   g.mCarga,
                   {g.xCarga, 0.0, 0.0},
                   inerciaCaja(g.mCarga, g.cargaLx, g.cargaLy, g.cargaLz) };

    // 3) Ala: dos paneles (placas planas) centrados a media semi-envergadura.
    //    El barrido y el diedro desplazan el centroide del panel en x y z.
    const double cMedia   = tiltrotorCuerdaMedia(g);
    const double semiSpan = g.envergadura / 2.0;
    for (int s = -1; s <= 1; s += 2) {
        comps[k++] = { "Panel de ala",
                       g.mAlaPanel,
                       { g.xAla - g.barrido*0.5, s * semiSpan * 0.5, -g.diedro*0.5 },
                       inerciaCaja(g.mAlaPanel, cMedia, semiSpan, 0.04) };
    }

    // 4) Deriva vertical (placa en el plano XZ, sube -> z negativo)
    comps[k++] = { "Deriva",
                   g.mDeriva,
                   { g.xDeriva, 0.0, -g.derivaAltura/2.0 },
                   inerciaCaja(g.mDeriva, g.derivaCuerda, 0.04, g.derivaAltura) };

    // 5) Estabilizador horizontal
    comps[k++] = { "Estabilizador",
                   g.mEstab,
                   { g.xEstab, 0.0, -0.05 },
                   inerciaCaja(g.mEstab, g.estabCuerda, g.estabEnverg, 0.04) };

    // 6) Tren de aterrizaje (debajo del fuselaje -> z positiva)
    comps[k++] = { "Tren de aterrizaje",
                   g.mTren,
                   { 0.0, 0.0, g.zTren },
                   inerciaCaja(g.mTren, g.trenLx, g.trenLy, g.trenLz) };

    // 7) Las 4 gondolas basculantes + sus 4 rotores.
    //    Gondola: cilindro cuyo eje sigue a la basculacion  -> rotarY(tau)
    //    Rotor  : disco cuyo eje de giro sigue al empuje    -> rotarY(tau - pi/2)
    //    Ambos con su CG desplazado del pivote por el mismo arco (rotarYVec),
    //    que es lo que corre el CG del vehiculo durante la transicion.
    for (int i = 0; i < 4; ++i) {
        const bool   del = (i < 2);
        const double xp  = del ? g.xGondFwd : g.xGondAft;
        const double yp  = del ? g.yGondFwd : g.yGondAft;
        const double sy  = (i % 2 == 0) ? +1.0 : -1.0;

        const Vec3 offGond  = rotarYVec(Vec3(-0.05, 0.0, 0.0), tau);
        const Vec3 offRotor = rotarYVec(Vec3(g.rotorOffset, 0.0, 0.0), tau);

        comps[k++] = { "Gondola basculante",
                       g.mGond,
                       { xp + offGond.x, sy*yp, g.zGond + offGond.z },
                       rotarY(inerciaCilindro(g.mGond, g.gondRadio, g.gondLargo), tau) };

        comps[k++] = { "Rotor",
                       g.mRotor,
                       { xp + offRotor.x, sy*yp, g.zGond + offRotor.z },
                       rotarY(inerciaDiscoZ(g.mRotor, g.rotorRadio), tau - M_PI/2.0) };
    }

    return combinarComponentes(comps, k, "convertiplano");
}


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 1 — Translational Kinematics
//
//   [pn_dot]   [cθcψ   sφsθcψ-cφsψ   cφsθcψ+sφsψ] [u]
//   [pe_dot] = [cθsψ   sφsθsψ+cφcψ   cφsθsψ-sφcψ] [v]
//   [pd_dot]   [-sθ    sφcθ           cφcθ        ] [w]
//
//   p_dot^v = R_b^v * v^b
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 translational_kinematics(const UAVState& s)
{
    const double cp = std::cos(s.phi),   sp = std::sin(s.phi);
    const double ct = std::cos(s.theta), st = std::sin(s.theta);
    const double cy = std::cos(s.psi),   sy = std::sin(s.psi);

    double pn_dot = (ct*cy)            * s.u
                  + (sp*st*cy - cp*sy) * s.v
                  + (cp*st*cy + sp*sy) * s.w;

    double pe_dot = (ct*sy)            * s.u
                  + (sp*st*sy + cp*cy) * s.v
                  + (cp*st*sy - sp*cy) * s.w;

    double pd_dot = (-st)              * s.u
                  + (sp*ct)            * s.v
                  + (cp*ct)            * s.w;

    return Vec3(pn_dot, pe_dot, pd_dot);
}


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 2 — Translational Dynamics
//
//   [u_dot]   [rv - qw]   1  [fx]
//   [v_dot] = [pw - ru] + -- [fy]
//   [w_dot]   [qu - pv]   m  [fz]
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 translational_dynamics(const UAVState& s,
                                    double fx, double fy, double fz,
                                    double mass)
{
    double u_dot = (s.r * s.v - s.q * s.w) + fx / mass;
    double v_dot = (s.p * s.w - s.r * s.u) + fy / mass;
    double w_dot = (s.q * s.u - s.p * s.v) + fz / mass;

    return Vec3(u_dot, v_dot, w_dot);
}


// ─────────────────────────────────────────────────────────────────────────────
// Arrastre aerodinamico simplificado (fuerza de entrada para el Bloque 2)
//
// Beard & McLain dejan fx,fy,fz como fuerzas de entrada externas (empuje +
// gravedad + aerodinamica); aca se agrega el termino de arrastre que faltaba:
// sin el, un empuje constante acelera sin limite y soltar el acelerador no
// frena nada (1a ley de Newton). Con drag, cada posicion de palanca tiene un
// punto de equilibrio real (empuje = drag) a una velocidad de crucero fija,
// como en un avion. Forma estandar de drag: F = -k * v * |v|, por eje del
// marco cuerpo (kx,ky,kz en N/(m/s)^2, calibrados por el llamador).
// ─────────────────────────────────────────────────────────────────────────────
struct DragCoeffs { double kx, ky, kz; };

inline Vec3 aeroDrag(const UAVState& s, const DragCoeffs& k)
{
    auto term = [](double v, double kv) { return -kv * v * std::abs(v); };
    return Vec3(term(s.u, k.kx), term(s.v, k.ky), term(s.w, k.kz));
}



// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 2b — Aerodinamica de ala fija (fuerzas de entrada del Bloque 2)
//
// aeroDrag() alcanza para un multirrotor, pero no para un vehiculo que TIENE
// ala: hace falta sustentacion, y sustentacion que dependa de la velocidad y
// del angulo de ataque, porque es exactamente lo que decide si una transicion
// de estacionario a crucero se completa o el vehiculo se cae. Este bloque
// implementa el modelo aerodinamico de Beard & McLain Cap. 4:
//
//   Datos de aire:   Va = |v_b|,  alpha = atan2(w,u),  beta = asin(v/Va)
//                    qbar = 1/2 * rho * Va^2   (presion dinamica)
//
//   CL(alpha) mezcla, con una sigmoide, el modelo LINEAL (CL0 + CLalpha*alpha,
//   valido antes de la entrada en perdida) con el de PLACA PLANA
//   (2*sign(alpha)*sin^2(alpha)*cos(alpha), valido despues) — eq. 4.9/4.10.
//   Sin esa mezcla el modelo lineal daria sustentacion absurda en vuelo
//   estacionario, donde alpha pasa por +-90 grados de forma rutinaria.
//
//   CD(alpha) = CDp + CL_lin^2 / (pi * e * AR)   — parasita + inducida (eq. 4.11)
//
//   Fuerzas al marco cuerpo (eq. 4.6), rotando (L,D) del marco viento al cuerpo:
//     fx = qbar*S*(-CD*cos(alpha) + CL*sin(alpha))
//     fy = qbar*S*( CY_beta*beta )
//     fz = qbar*S*(-CD*sin(alpha) - CL*cos(alpha))
//
// Coeficientes por defecto: los del Aerosonde (Beard & McLain, Apendice E),
// que es un UAV de la misma clase de tamano que los de este simulador.
// ─────────────────────────────────────────────────────────────────────────────
struct AeroCoeffs {
    double S, b, c;          // area alar [m^2], envergadura [m], cuerda media [m]
    double rho;              // densidad del aire [kg/m^3]
    double e;                // eficiencia de Oswald
    double Mblend, alpha0;   // sigmoide de entrada en perdida (eq. 4.10)

    double CL0, CLalpha, CDp;
    double CYbeta, CYp, CYr;

    double Cl0, Clbeta, Clp, Clr;      // momento de alabeo (roll)
    double Cm0, Cmalpha, Cmq;          // momento de cabeceo (pitch)
    double Cn0, Cnbeta, Cnp, Cnr;      // momento de guinada (yaw)

    double Cldelta_a, Cmdelta_e, Cndelta_r;   // autoridad de las superficies
};

// Coeficientes de referencia (Aerosonde). S, b y c los pone el vehiculo, para
// que la aerodinamica use la MISMA ala que el Bloque 0 y que el dibujo 3D.
inline AeroCoeffs aeroCoeffsReferencia(double S, double b, double c,
                                       double rho = 1.2682)
{
    AeroCoeffs a{};
    a.S = S; a.b = b; a.c = c; a.rho = rho;
    a.e = 0.9;  a.Mblend = 50.0;  a.alpha0 = 0.47;

    a.CL0 = 0.28;  a.CLalpha = 3.45;  a.CDp = 0.043;
    a.CYbeta = -0.98; a.CYp = 0.0; a.CYr = 0.0;

    a.Cl0 = 0.0;  a.Clbeta = -0.12;  a.Clp = -0.26;  a.Clr = 0.14;
    a.Cm0 = 0.02; a.Cmalpha = -0.38; a.Cmq = -3.60;
    a.Cn0 = 0.0;  a.Cnbeta = 0.25;   a.Cnp = 0.022;  a.Cnr = -0.35;

    a.Cldelta_a = 0.08;  a.Cmdelta_e = -0.50;  a.Cndelta_r = -0.032;
    return a;
}

// Datos de aire derivados del estado (marco cuerpo, sin viento: Va = |v_b|)
struct AirData {
    double Va;       // velocidad aerodinamica [m/s]
    double alpha;    // angulo de ataque [rad]
    double beta;     // angulo de resbalamiento [rad]
    double qbar;     // presion dinamica [Pa]
};

inline AirData computeAirData(const UAVState& s, double rho)
{
    AirData d;
    d.Va    = std::sqrt(s.u*s.u + s.v*s.v + s.w*s.w);
    d.alpha = std::atan2(s.w, s.u);
    d.beta  = (d.Va > 1e-6) ? std::asin(std::max(-1.0, std::min(1.0, s.v / d.Va))) : 0.0;
    d.qbar  = 0.5 * rho * d.Va * d.Va;
    return d;
}

// Sigmoide de mezcla lineal <-> placa plana (eq. 4.10). Los exponentes se
// acotan para que exp() no desborde con alpha cerca de +-pi.
inline double stallBlend(double alpha, double M, double alpha0)
{
    auto ex = [](double x) { return std::exp(std::max(-60.0, std::min(60.0, x))); };
    const double e1 = ex(-M * (alpha - alpha0));
    const double e2 = ex( M * (alpha + alpha0));
    return (1.0 + e1 + e2) / ((1.0 + e1) * (1.0 + e2));
}

inline double liftCoefficient(double alpha, const AeroCoeffs& a)
{
    const double sg   = stallBlend(alpha, a.Mblend, a.alpha0);
    const double lin  = a.CL0 + a.CLalpha * alpha;
    const double sgn  = (alpha >= 0.0) ? 1.0 : -1.0;
    const double flat = 2.0 * sgn * std::sin(alpha) * std::sin(alpha) * std::cos(alpha);
    return (1.0 - sg) * lin + sg * flat;
}

inline double dragCoefficient(double alpha, const AeroCoeffs& a)
{
    const double AR  = (a.S > 1e-9) ? (a.b * a.b / a.S) : 1.0;
    const double lin = a.CL0 + a.CLalpha * alpha;
    return a.CDp + lin * lin / (M_PI * a.e * AR);
}

// Fuerzas aerodinamicas en el marco cuerpo (eq. 4.6) — entran al Bloque 2
// sumadas al empuje y a la gravedad, igual que aeroDrag().
inline Vec3 aeroForces(const AirData& d, const AeroCoeffs& a)
{
    if (d.Va < 0.5) return Vec3(0, 0, 0);   // sin flujo util no hay aerodinamica

    const double CL = liftCoefficient(d.alpha, a);
    const double CD = dragCoefficient(d.alpha, a);
    const double ca = std::cos(d.alpha), sa = std::sin(d.alpha);
    const double qS = d.qbar * a.S;

    return Vec3( qS * (-CD*ca + CL*sa),
                 qS * ( a.CYbeta * d.beta ),
                 qS * (-CD*sa - CL*ca) );
}

// Momentos aerodinamicos PROPIOS DE LA CELULA (eq. 4.7/4.19): estabilidad
// estatica (Cm_alpha, Cn_beta, Cl_beta) + amortiguamiento por velocidad
// angular (Clp, Cmq, Cnr). NO incluye las deflexiones de mando: la autoridad
// de aleron/elevador/timon entra por tiltrotorAuthority() (Bloque 4b), para
// no contar dos veces la misma superficie.
//
// Es lo que le da al vehiculo su dinamica rotacional "natural": aunque no se
// toque nada, el ala se opone a alabear, la cola se opone a cabecear y la
// deriva alinea la nariz con el viento relativo. En estacionario qbar ~ 0 y
// todo esto desaparece — ahi el unico que estabiliza es el control de rotores.
inline Vec3 aeroMomentsAirframe(const UAVState& s, const AirData& d,
                                const AeroCoeffs& a)
{
    if (d.Va < 0.5) return Vec3(0, 0, 0);

    const double kb = a.b / (2.0 * d.Va);   // adimensionaliza p y r
    const double kc = a.c / (2.0 * d.Va);   // adimensionaliza q
    const double qS = d.qbar * a.S;

    const double l = qS * a.b * ( a.Cl0 + a.Clbeta*d.beta
                                + a.Clp*kb*s.p + a.Clr*kb*s.r );
    const double m = qS * a.c * ( a.Cm0 + a.Cmalpha*d.alpha
                                + a.Cmq*kc*s.q );
    const double n = qS * a.b * ( a.Cn0 + a.Cnbeta*d.beta
                                + a.Cnp*kb*s.p + a.Cnr*kb*s.r );
    return Vec3(l, m, n);
}


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 3 — Rotational Kinematics
//
//   [phi_dot  ]   [1  sin(phi)*tan(theta)  cos(phi)*tan(theta)] [p]
//   [theta_dot] = [0  cos(phi)            -sin(phi)           ] [q]
//   [psi_dot  ]   [0  sin(phi)/cos(theta)  cos(phi)/cos(theta)] [r]
//
// WARNING: singularity at theta = ±90 deg (gimbal lock)
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 rotational_kinematics(const UAVState& s)
{
    const double sp = std::sin(s.phi), cp = std::cos(s.phi);
    const double ct = std::cos(s.theta), tt = std::tan(s.theta);

    if (std::abs(ct) < 1e-6)
        std::cerr << "[WARNING] Gimbal lock: theta near ±90 deg\n";

    const double sc = 1.0 / ct;  // sec(theta)

    double phi_dot   = s.p + sp*tt * s.q + cp*tt * s.r;
    double theta_dot =       cp    * s.q - sp    * s.r;
    double psi_dot   =       sp*sc * s.q + cp*sc * s.r;

    return Vec3(phi_dot, theta_dot, psi_dot);
}


// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 4 — Rotational Dynamics
//
//   p_dot = Γ1*p*q  - Γ2*q*r  + Γ3*l  + Γ4*n
//   q_dot = Γ5*p*r  - Γ6*(p²-r²)       + m/Jy
//   r_dot = Γ7*p*q  - Γ1*q*r  + Γ4*l  + Γ8*n
//
// Γ constants derived from the inertia tensor (see Beard & McLain eq. 3.17)
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 rotational_dynamics(const UAVState& s,
                                 double l, double m_torque, double n,
                                 const Inertia& J)
{
    const double G  = J.Jx * J.Jz - J.Jxz * J.Jxz;
    const double G1 = J.Jxz * (J.Jx - J.Jy + J.Jz) / G;
    const double G2 = (J.Jz * (J.Jz - J.Jy) + J.Jxz * J.Jxz) / G;
    const double G3 = J.Jz  / G;
    const double G4 = J.Jxz / G;
    const double G5 = (J.Jz - J.Jx) / J.Jy;
    const double G6 = J.Jxz / J.Jy;
    const double G7 = ((J.Jx - J.Jy) * J.Jx + J.Jxz * J.Jxz) / G;
    const double G8 = J.Jx / G;

    double p_dot = G1*s.p*s.q  - G2*s.q*s.r  + G3*l + G4*n;
    double q_dot = G5*s.p*s.r  - G6*(s.p*s.p - s.r*s.r)  + m_torque / J.Jy;
    double r_dot = G7*s.p*s.q  - G1*s.q*s.r  + G4*l + G8*n;

    return Vec3(p_dot, q_dot, r_dot);
}



// ─────────────────────────────────────────────────────────────────────────────
// BLOQUE 4b — Control de actitud del convertiplano: de que puede tirar cada
// modo de vuelo para generar TORQUE, y cuanto torque pide el piloto.
//
// Los otros dos vehiculos del simulador comandan p,q,r directamente con el
// teclado y saltean el Bloque 4. El convertiplano no puede: su autoridad de
// control cambia por completo durante la transicion, y esa es justamente su
// fisica interesante. Aca p,q,r SI salen del Bloque 4, alimentado por:
//
//   torque total = torque de mando (saturado por la autoridad disponible)
//                + momentos aerodinamicos propios de la celula (Bloque 2b)
//
// De donde sale la autoridad, para una basculacion sigma:
//
//   a) EMPUJE DIFERENCIAL entre rotores. Cada rotor empuja sobre el eje de su
//      gondola, n = (sin s, 0, -cos s). Un diferencial dT entre rotores da:
//        - componente vertical (-cos s): rotores a +-y  -> ALABEO
//                                        rotores a +-x  -> CABECEO
//        - componente longitudinal (sin s): rotores a +-y -> GUINADA
//      O sea: en estacionario el diferencial alabea y cabecea; en modo avion
//      el MISMO diferencial guina (empuje asimetrico, como un bimotor).
//
//   b) BASCULACION DIFERENCIAL. Inclinar unas gondolas mas que otras da una
//      fuerza sobre la perpendicular t = (cos s, 0, sin s):
//        - en estacionario t = +x, aplicada a +-y  -> GUINADA
//        - en crucero      t = +z, aplicada a +-y  -> ALABEO
//      Exactamente complementaria a (a): entre las dos, los tres ejes quedan
//      cubiertos en todo el rango de la transicion.
//
//   c) SUPERFICIES AERODINAMICAS (aleron, elevador, timon). Escalan con qbar,
//      asi que valen cero en estacionario y son el mando principal en crucero
//      — el reparto rotores/superficies se hace solo, sin logica de modos.
//
// El piloto comanda TASAS (p,q,r deseadas), no torques: un lazo proporcional
// pide el torque que llevaria la tasa al valor comandado con una constante de
// tiempo tau, y despues lo satura contra la autoridad que realmente hay.
// Cuando satura, el vehiculo responde lento — que es lo que de verdad pasa
// cuando se pide mas mando del que el modo de vuelo puede dar.
// ─────────────────────────────────────────────────────────────────────────────
struct TiltRotorLayout {
    double armX_fwd, armX_aft;   // |x| de los rotores respecto al CG [m]
    double armY_fwd, armY_aft;   // |y| de los rotores respecto al CG [m]
    double diffFrac;             // fraccion de T/4 usable como diferencial
    double diffTiltMax;          // basculacion diferencial maxima [rad]
};

struct ControlAuthority {
    double l, m, n;              // |torque| maximo disponible por eje [N*m]
};

struct ControlTorques {
    double l, m, n;              // torque de mando aplicado [N*m]
    double satL, satM, satN;     // fraccion de autoridad usada [0..1] (1 = saturado)
};

inline ControlAuthority tiltrotorAuthority(double thrust, double sigma,
                                           const TiltRotorLayout& lay,
                                           const AeroCoeffs& a, double qbar)
{
    const double dT   = lay.diffFrac * std::abs(thrust) / 4.0;   // N por rotor
    const double cs   = std::abs(std::cos(sigma));
    const double sn   = std::abs(std::sin(sigma));
    const double armY = 0.5 * (lay.armY_fwd + lay.armY_aft);
    const double armX = 0.5 * (lay.armX_fwd + lay.armX_aft);
    const double Ft   = std::abs(thrust) * std::sin(lay.diffTiltMax);  // por basculacion dif.

    // (a) empuje diferencial + (b) basculacion diferencial
    const double l_rot = 2.0*dT*cs*armY + Ft*sn*armY;
    const double m_rot = 2.0*dT*cs*armX;
    const double n_rot = 2.0*dT*sn*armY + Ft*cs*armY;

    // (c) superficies de mando
    const double l_aer = qbar * a.S * a.b * std::abs(a.Cldelta_a);
    const double m_aer = qbar * a.S * a.c * std::abs(a.Cmdelta_e);
    const double n_aer = qbar * a.S * a.b * std::abs(a.Cndelta_r);

    return { l_rot + l_aer, m_rot + m_aer, n_rot + n_aer };
}

// Lazo proporcional de tasa: torque = J * (tasa_comandada - tasa) / tau,
// saturado por la autoridad disponible. Escribir la ganancia como J/tau (y no
// como un numero suelto) hace que la respuesta sea la MISMA aunque el tensor
// de inercia cambie con la basculacion: sin saturar, p -> p_cmd con constante
// de tiempo tau, sea cual sea el modo de vuelo.
inline ControlTorques tiltrotorRateControl(const UAVState& s,
                                            double p_cmd, double q_cmd, double r_cmd,
                                            const Inertia& J,
                                            const ControlAuthority& au,
                                            double tauP, double tauQ, double tauR)
{
    auto sat = [](double v, double lim, double& frac) {
        if (lim <= 1e-9) { frac = 0.0; return 0.0; }
        frac = std::min(1.0, std::abs(v) / lim);
        return std::max(-lim, std::min(lim, v));
    };

    ControlTorques t{};
    t.l = sat(J.Jx * (p_cmd - s.p) / tauP, au.l, t.satL);
    t.m = sat(J.Jy * (q_cmd - s.q) / tauQ, au.m, t.satM);
    t.n = sat(J.Jz * (r_cmd - s.r) / tauR, au.n, t.satN);
    return t;
}


// ─────────────────────────────────────────────────────────────────────────────
// Euler integration — advances the full 12-state UAV by one timestep dt
// (metodo de 1er orden; ver integrate_rk4 mas abajo para mayor precision)
// ─────────────────────────────────────────────────────────────────────────────
inline UAVState integrate_euler(const UAVState& s,
                                 double fx, double fy, double fz,
                                 double l,  double m_torque, double n,
                                 const Inertia& J,
                                 double mass, double dt)
{
    UAVState next = s;

    Vec3 pos_dot   = translational_kinematics(s);                // Bloque 1
    Vec3 vel_dot   = translational_dynamics(s, fx,fy,fz, mass);  // Bloque 2
    Vec3 euler_dot = rotational_kinematics(s);                   // Bloque 3
    Vec3 rate_dot  = rotational_dynamics(s, l, m_torque, n, J);  // Bloque 4

    next.pn  += pos_dot.x   * dt;
    next.pe  += pos_dot.y   * dt;
    next.pd  += pos_dot.z   * dt;

    next.u   += vel_dot.x   * dt;
    next.v   += vel_dot.y   * dt;
    next.w   += vel_dot.z   * dt;

    next.phi   += euler_dot.x * dt;
    next.theta += euler_dot.y * dt;
    next.psi   += euler_dot.z * dt;

    next.p   += rate_dot.x  * dt;
    next.q   += rate_dot.y  * dt;
    next.r   += rate_dot.z  * dt;

    return next;
}


// ─────────────────────────────────────────────────────────────────────────────
// Runge-Kutta 4 (RK4) — integrador de 4to orden para el mismo estado de 12
// componentes. Evalua los Bloques 1-4 (la derivada completa del estado) en
// 4 puntos por paso (k1..k4) y los combina con los pesos clasicos 1-2-2-1.
// Fuerzas y torques (fx,fy,fz,l,m_torque,n) se asumen constantes durante el
// paso dt — igual que en integrate_euler; solo cambia el orden de precision
// de la integracion, no el modelo de fuerzas.
// ─────────────────────────────────────────────────────────────────────────────

// Derivada completa del estado (Bloques 1-4 evaluados en un mismo instante)
struct StateDot {
    double pn, pe, pd;
    double u, v, w;
    double phi, theta, psi;
    double p, q, r;
};

inline StateDot state_derivative(const UAVState& s,
                                  double fx, double fy, double fz,
                                  double l,  double m_torque, double n,
                                  const Inertia& J, double mass)
{
    Vec3 pos_dot   = translational_kinematics(s);                // Bloque 1
    Vec3 vel_dot   = translational_dynamics(s, fx,fy,fz, mass);  // Bloque 2
    Vec3 euler_dot = rotational_kinematics(s);                   // Bloque 3
    Vec3 rate_dot  = rotational_dynamics(s, l, m_torque, n, J);  // Bloque 4

    return { pos_dot.x, pos_dot.y, pos_dot.z,
             vel_dot.x, vel_dot.y, vel_dot.z,
             euler_dot.x, euler_dot.y, euler_dot.z,
             rate_dot.x, rate_dot.y, rate_dot.z };
}

// Estado + h * derivada — construye los estados intermedios s2,s3,s4 de RK4
inline UAVState state_add_scaled(const UAVState& s, const StateDot& d, double h)
{
    UAVState out = s;
    out.pn    += h * d.pn;
    out.pe    += h * d.pe;
    out.pd    += h * d.pd;
    out.u     += h * d.u;
    out.v     += h * d.v;
    out.w     += h * d.w;
    out.phi   += h * d.phi;
    out.theta += h * d.theta;
    out.psi   += h * d.psi;
    out.p     += h * d.p;
    out.q     += h * d.q;
    out.r     += h * d.r;
    return out;
}

// Combina k1..k4 con los pesos clasicos de RK4 (1-2-2-1) sobre el estado base.
// Compartido con simulateStep() (simulate_step.h), que evalua las mismas
// 4 etapas pero con dinamica rotacional de tensor de inercia completo.
inline UAVState rk4_combine(const UAVState& s,
                             const StateDot& k1, const StateDot& k2,
                             const StateDot& k3, const StateDot& k4,
                             double dt)
{
    UAVState next = s;
    next.pn    += (dt / 6.0) * (k1.pn    + 2.0*k2.pn    + 2.0*k3.pn    + k4.pn);
    next.pe    += (dt / 6.0) * (k1.pe    + 2.0*k2.pe    + 2.0*k3.pe    + k4.pe);
    next.pd    += (dt / 6.0) * (k1.pd    + 2.0*k2.pd    + 2.0*k3.pd    + k4.pd);
    next.u     += (dt / 6.0) * (k1.u     + 2.0*k2.u     + 2.0*k3.u     + k4.u);
    next.v     += (dt / 6.0) * (k1.v     + 2.0*k2.v     + 2.0*k3.v     + k4.v);
    next.w     += (dt / 6.0) * (k1.w     + 2.0*k2.w     + 2.0*k3.w     + k4.w);
    next.phi   += (dt / 6.0) * (k1.phi   + 2.0*k2.phi   + 2.0*k3.phi   + k4.phi);
    next.theta += (dt / 6.0) * (k1.theta + 2.0*k2.theta + 2.0*k3.theta + k4.theta);
    next.psi   += (dt / 6.0) * (k1.psi   + 2.0*k2.psi   + 2.0*k3.psi   + k4.psi);
    next.p     += (dt / 6.0) * (k1.p     + 2.0*k2.p     + 2.0*k3.p     + k4.p);
    next.q     += (dt / 6.0) * (k1.q     + 2.0*k2.q     + 2.0*k3.q     + k4.q);
    next.r     += (dt / 6.0) * (k1.r     + 2.0*k2.r     + 2.0*k3.r     + k4.r);
    return next;
}

inline UAVState integrate_rk4(const UAVState& s,
                               double fx, double fy, double fz,
                               double l,  double m_torque, double n,
                               const Inertia& J,
                               double mass, double dt)
{
    StateDot k1 = state_derivative(s, fx,fy,fz, l,m_torque,n, J, mass);
    UAVState s2 = state_add_scaled(s, k1, dt / 2.0);

    StateDot k2 = state_derivative(s2, fx,fy,fz, l,m_torque,n, J, mass);
    UAVState s3 = state_add_scaled(s, k2, dt / 2.0);

    StateDot k3 = state_derivative(s3, fx,fy,fz, l,m_torque,n, J, mass);
    UAVState s4 = state_add_scaled(s, k3, dt);

    StateDot k4 = state_derivative(s4, fx,fy,fz, l,m_torque,n, J, mass);

    return rk4_combine(s, k1, k2, k3, k4, dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// Conversiones de actitud: Euler <-> Cuaternion <-> Matriz de rotacion
//
// Cuaternion unitario e_b^i = (e0,e1,e2,e3): rotacion de cuerpo a inercial.
// e0 es la parte escalar, (e1,e2,e3) la parte vectorial.
//
// Formulas: euler_to_quaternions.jpeg y quaternion_to_rotationMatrix.jpeg
// Implementadas (no inline) en physics_UAV.cpp — son utilidades de conversion
// de actitud, separadas del lazo de integracion (integrate_euler), que sigue
// trabajando en angulos de Euler.
// ─────────────────────────────────────────────────────────────────────────────
struct Quaternion {
    double e0, e1, e2, e3;
};

struct Mat3x3 {
    double m[3][3];
};

Quaternion eulerToQuaternion(double phi, double theta, double psi);
Vec3       quaternionToEuler(const Quaternion& e);          // devuelve (phi, theta, psi)
Mat3x3     quaternionToRotationMatrix(const Quaternion& e); // R_b^i: cuerpo -> inercial


// ─────────────────────────────────────────────────────────────────────────────
// Piso — restriccion de contacto en el plano X-Y (colision con el suelo)
//
// El suelo es el plano horizontal del marco inercial NED, ubicado en
// pd = pisoPD (por defecto 0.0 = nivel de referencia). Como pd es positivo
// hacia abajo, cruzar pd > pisoPD significa que el dron atraveso el piso.
// ─────────────────────────────────────────────────────────────────────────────
struct GroundContact {
    bool   colision;
    double penetracion;   // cuanto se paso del piso (m); 0 si no hay colision
};

inline GroundContact checkGroundCollision(const UAVState& s, double pisoPD = 0.0)
{
    double pen = s.pd - pisoPD;
    return { pen > 0.0, pen > 0.0 ? pen : 0.0 };
}

// Respuesta inelastica minima: clava el dron al piso y anula su velocidad
// (frena en seco — no hay rebote ni deformacion, es una restriccion dura)
inline UAVState resolveGroundCollision(UAVState s, double pisoPD = 0.0)
{
    if (s.pd > pisoPD) {
        s.pd = pisoPD;
        s.u = s.v = s.w = 0.0;
    }
    return s;
}


// ─────────────────────────────────────────────────────────────────────────────
// Obstaculo — bloque rectangular fijo, colision AABB (caja alineada a ejes)
//
// Definido por su centro en el plano horizontal (pnCenter, peCenter) y sus
// dimensiones reales: ancho en pn [m], profundidad en pe [m] y altura desde
// el piso [m]. Igual que el piso, es una restriccion dura, sin deformacion
// ni rebote; el dron se trata como un punto (su CG) para la colision, misma
// simplificacion que usa checkGroundCollision().
// ─────────────────────────────────────────────────────────────────────────────
struct BoxObstacle {
    double pnCenter, peCenter;  // centro del bloque en el plano horizontal [m]
    double sizePn, sizePe;      // dimensiones del bloque en pn (X) y pe (Y) [m]
    double height;               // altura del bloque desde el piso [m]
    double pisoPD;                // pd del piso sobre el que se apoya el bloque
};

struct ObstacleContact {
    bool colision;
};

inline ObstacleContact checkObstacleCollision(const UAVState& s, const BoxObstacle& box)
{
    const double halfPn = box.sizePn / 2.0;
    const double halfPe = box.sizePe / 2.0;

    const bool dentroPn  = std::abs(s.pn - box.pnCenter) <= halfPn;
    const bool dentroPe  = std::abs(s.pe - box.peCenter) <= halfPe;
    const bool dentroAlt = (s.pd <= box.pisoPD) && (s.pd >= box.pisoPD - box.height);

    return { dentroPn && dentroPe && dentroAlt };
}

// Respuesta inelastica: empuja al dron fuera de la caja por el eje de menor
// penetracion (MTV — minimum translation vector) y anula su velocidad,
// misma filosofia "solido, sin rebote" que resolveGroundCollision pero en
// los 3 ejes, para que el bloque sea realmente solido y no se pueda
// atravesar sosteniendo el acelerador.
inline UAVState resolveObstacleCollision(UAVState s, const BoxObstacle& box)
{
    const double halfPn = box.sizePn / 2.0;
    const double halfPe = box.sizePe / 2.0;
    const double top    = box.pisoPD - box.height;   // pd del techo (arriba -> pd menor)
    const double bottom = box.pisoPD;                // pd del piso del bloque

    const double overlapPn  = halfPn - std::abs(s.pn - box.pnCenter);
    const double overlapPe  = halfPe - std::abs(s.pe - box.peCenter);
    const double overlapAlt = (bottom - s.pd < s.pd - top) ? (bottom - s.pd) : (s.pd - top);

    if (overlapPn <= overlapPe && overlapPn <= overlapAlt) {
        s.pn = (s.pn >= box.pnCenter) ? box.pnCenter + halfPn : box.pnCenter - halfPn;
    } else if (overlapPe <= overlapAlt) {
        s.pe = (s.pe >= box.peCenter) ? box.peCenter + halfPe : box.peCenter - halfPe;
    } else {
        s.pd = (s.pd <= (top + bottom) / 2.0) ? top : bottom;
    }

    s.u = s.v = s.w = 0.0;
    return s;
}

#endif // PHYSICS_UAV_H
