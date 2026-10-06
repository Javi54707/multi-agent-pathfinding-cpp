#include "tecnico.hpp"
#include "motorlib/util.h"
#include <iostream>
#include <queue>
#include <set>
#include <algorithm>

using namespace std;

// =========================================================================
// ÁREA DE IMPLEMENTACIÓN DEL ESTUDIANTE
// =========================================================================

Action ComportamientoTecnico::think(Sensores sensores) {
  Action accion = IDLE;


  // Decisión del agente según el nivel
  switch (sensores.nivel) {
    case 0: accion = ComportamientoTecnicoNivel_0(sensores); break;
    case 1: accion = ComportamientoTecnicoNivel_1(sensores); break;
    case 2: accion = ComportamientoTecnicoNivel_2(sensores); break;
    case 3: accion = ComportamientoTecnicoNivel_3(sensores); break;
    case 4: accion = ComportamientoTecnicoNivel_4(sensores); break;
    case 5: accion = ComportamientoTecnicoNivel_5(sensores); break;
    case 6: accion = ComportamientoTecnicoNivel_6(sensores); break;
  }

  return accion;
}


// Niveles del técnico
Action ComportamientoTecnico::ComportamientoTecnicoNivel_0(Sensores sensores) {
  Action accion = IDLE;

  ActualizarMapa(sensores);

  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;
  if (sensores.superficie[0] == 'U') return IDLE;

  mapa_visitas[{sensores.posF, sensores.posC}]++;

  int prioridades[3];
  ubicacion ubicaciones[3];
  ubicacion actual = {sensores.posF, sensores.posC, sensores.rumbo};
  Orientacion izq = static_cast<Orientacion>((sensores.rumbo + 7) % 8);
  Orientacion der = static_cast<Orientacion>((sensores.rumbo + 1) % 8);
  ubicacion mira_izq = {sensores.posF, sensores.posC, izq};
  ubicacion mira_der = {sensores.posF, sensores.posC, der};

  ubicaciones[0] = mira_izq;
  ubicaciones[1] = actual;
  ubicaciones[2] = mira_der;

  for (int i = 0; i < 3; i++) {
    ubicacion destino = Delante(ubicaciones[i]);
    unsigned char sup = sensores.superficie[i+1];

    if (sensores.agentes[i+1] != '_') {
        // Le asignamos 5 visitas (o mantenemos si ya tenía más) para espantar al radar
        mapa_visitas[{destino.f, destino.c}] = mapa_visitas[{destino.f, destino.c}] + 5;
    }

    if (AndarViable0(ubicaciones[i], tiene_zapatillas) && sup == 'U' && sensores.agentes[i+1] == '_')
      prioridades[i] = -999999 + mapa_visitas[{destino.f, destino.c}];
    else if (AndarViable0(ubicaciones[i], tiene_zapatillas) && (sup == 'D' && !tiene_zapatillas) && sensores.agentes[i+1] == '_')
      prioridades[i] = -500000 + mapa_visitas[{destino.f, destino.c}];
    else if (AndarViable0(ubicaciones[i], tiene_zapatillas) && (sup == 'C' || (sup == 'D' && tiene_zapatillas)) && sensores.agentes[i+1] == '_')
      prioridades[i] = mapa_visitas[{destino.f, destino.c}];
    else
      prioridades[i] = 999999;
  }

  int min_frente = *min_element(prioridades, prioridades + 3); 

  if (min_frente > 0) {
    
    // EXIGIMOS que el radar encuentre algo mejor que lo que tenemos enfrente
    int mejor_visita_radar = min_frente; 
    int mejor_giro = -1; // -1 significa "no forzar giro"

    // Escaneamos las 7 direcciones periféricas
    for (int g = 1; g < 8; g++) {
      Orientacion ori = static_cast<Orientacion>((sensores.rumbo + g) % 8);
      ubicacion mira = {sensores.posF, sensores.posC, ori};

      if (AndarViable0(mira, tiene_zapatillas)) {
        int vis = mapa_visitas[{Delante(mira).f, Delante(mira).c}];

        // Solo sobrescribimos si está MENOS visitado que lo de enfrente
        if (vis < mejor_visita_radar) {
          mejor_visita_radar = vis;
          // Si la ruta buena está a la derecha (1, 2, 3), forzamos TURN_SR (índice 2)
          if (g >= 1 && g <= 3) {
            mejor_giro = 2; 
          } 
          // Si está a la izquierda o atrás (4, 5, 6, 7), forzamos TURN_SL (índice 0)
          else {
            mejor_giro = 0; 
          }
        }
      }
    }

    // Si el radar encontró una ruta ESTRICTAMENTE MEJOR, forzamos la decisión
    if (mejor_giro != -1) {
      prioridades[mejor_giro] = -10000;
    }
  }

  int min = distance(prioridades, min_element(prioridades, prioridades + 3));

  switch (min) {
  case 0:
    accion = TURN_SL;
    break;
  case 1:
    accion = WALK;
    break;
  case 2:
    accion = TURN_SR;
    break;
  }

  last_action = accion;
  return accion;
}

/**
 * @brief Comprueba si una celda es de tipo camino transitable.
 * @param c Carácter que representa el tipo de superficie.
 * @return true si es camino ('C'), zapatillas ('D') o meta ('U').
 */
bool ComportamientoTecnico::es_camino(unsigned char c) const {
  return (c == 'C' || c == 'D' || c == 'U');
}


/**
 * @brief Comportamiento reactivo del técnico para el Nivel 1.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoTecnico::ComportamientoTecnicoNivel_1(Sensores sensores) {
  Action accion = IDLE;

  ActualizarMapa(sensores);

  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;

  mapa_visitas[{sensores.posF, sensores.posC}]++;

  int prioridades[3];
  ubicacion ubicaciones[3];
  ubicacion actual = {sensores.posF, sensores.posC, sensores.rumbo};
  Orientacion izq = static_cast<Orientacion>((sensores.rumbo + 7) % 8);
  Orientacion der = static_cast<Orientacion>((sensores.rumbo + 1) % 8);
  ubicacion mira_izq = {sensores.posF, sensores.posC, izq};
  ubicacion mira_der = {sensores.posF, sensores.posC, der};

  ubicaciones[0] = mira_izq;
  ubicaciones[1] = actual;
  ubicaciones[2] = mira_der;

  // --- BUCLE DE ANDAR ---
  for (int i = 0; i < 3; i++) {
    ubicacion destino = Delante(ubicaciones[i]);
    unsigned char sup = sensores.superficie[i+1];

    if (sensores.agentes[i+1] != '_') {
      mapa_visitas[{destino.f, destino.c}] += 5;
    }

    int visitas = mapa_visitas[{destino.f, destino.c}];

    if (!AndarViable(ubicaciones[i], tiene_zapatillas) || sensores.agentes[i+1] != '_') {
      prioridades[i] = 999999;
    } 
    else {
      int base_prioridad = visitas * 10000;
      
      if (sup == 'D' && !tiene_zapatillas) {
        prioridades[i] = base_prioridad - 50000;
      } 
      else if (sup == 'C') {
        prioridades[i] = base_prioridad - 30000;
      }
      else if (sup == 'S') {
        prioridades[i] = base_prioridad - 20000;
      }
      else if (sup == 'A') {
        prioridades[i] = base_prioridad + 20000;
      }
      else {
        // Hierba o Zapatillas ya conseguidas
        prioridades[i] = base_prioridad;
      }
    }
  }

  int inercia = (sensores.nivel == 6) ? 15000 : 1;

  if (prioridades[1] < 999999) prioridades[1] -= inercia; // WALK

  int min_frente = *min_element(prioridades, prioridades + 3); 

  if (min_frente >= 0) {
    int mejor_prio_radar = min_frente; 
    int mejor_giro = -1; 

    for (int g = 1; g < 8; g++) {
      Orientacion ori = static_cast<Orientacion>((sensores.rumbo + g) % 8);
      ubicacion mira = {sensores.posF, sensores.posC, ori};

      if (AndarViable(mira, tiene_zapatillas)) {
        ubicacion dest_radar = Delante(mira);
        unsigned char sup = mapaResultado[dest_radar.f][dest_radar.c];
        
        int prio_lateral = mapa_visitas[{dest_radar.f, dest_radar.c}] * 10000;
        
        if (sup == 'D' && !tiene_zapatillas) prio_lateral -= 50000;
        else if (sup == 'C') prio_lateral -= 30000;
        else if (sup == 'S') prio_lateral -= 20000;
        else if (sup == 'A') prio_lateral += 20000;

        if (prio_lateral < mejor_prio_radar) {
          mejor_prio_radar = prio_lateral;
          if (g >= 1 && g <= 3) mejor_giro = 2; 
          else mejor_giro = 0; 
        }
      }
    }

    if (mejor_giro != -1) {
      prioridades[mejor_giro] = -1000000; 
    }
  }

  int min = distance(prioridades, min_element(prioridades, prioridades + 3));

  // Si acabamos de girar a un lado, prohibimos girar al lado contrario inmediatamente
  // para romper cualquier bucle de Izquierda-Derecha.
  if (min == 0 && last_action == TURN_SR) {
    prioridades[0] = 999999;
    min = distance(prioridades, min_element(prioridades, prioridades + 3));
  } else if (min == 2 && last_action == TURN_SL) {
    prioridades[2] = 999999;
    min = distance(prioridades, min_element(prioridades, prioridades + 3));
  }

  switch (min) {
  case 0:
    accion = TURN_SL;
    break;
  case 1:
    accion = WALK;
    break;
  case 2:
    accion = TURN_SR;
    break;
  }

  last_action = accion;
  return accion;
}

/**
 * @brief Comportamiento del técnico para el Nivel 2.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoTecnico::ComportamientoTecnicoNivel_2(Sensores sensores) {
  // Estrategia de Evasión Táctica:
  // Me quedo quieto (IDLE) para no estorbar, a menos que detecte al Ingeniero ('i')
  // justo delante de mí, a punto de atropellarme.
  
  bool ingenieroCerca = false;
  for (int i = 1; i <= 3; i++) {
    if (sensores.agentes[i] == 'i') {
      ingenieroCerca = true;
      break;
    }
  }

  // Si el Ingeniero viene hacia mí en modo "apartate que voy", me quito de en medio
  if (ingenieroCerca) {
    ubicacion actual = {sensores.posF, sensores.posC, sensores.rumbo};
    if (AndarViable(actual, tiene_zapatillas)) {
      return WALK;   
    } else {
      return TURN_SR; 
    }
  }

  return IDLE;
}

/**
 * @brief Comportamiento del técnico para el Nivel 3.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoTecnico::ComportamientoTecnicoNivel_3(Sensores sensores) {
  Action accion = IDLE;

  if (!hayPlan) {
    EstadoT inicio;
    inicio.site.f = sensores.posF;
    inicio.site.c = sensores.posC;
    inicio.site.brujula = sensores.rumbo;
    inicio.zapatillas = tiene_zapatillas;

    EstadoT final;
    final.site.f = sensores.BelPosF;
    final.site.c = sensores.BelPosC;

    plan = AEstrellaTecnico(inicio, final, mapaResultado, mapaCotas);
    VisualizaPlan(inicio.site, plan);

    hayPlan = !plan.empty();
  }

  if (hayPlan && !plan.empty()) {
    accion = plan.front();
    plan.pop_front();
    
    if (plan.empty()) {
      hayPlan = false;
    }
  }

  last_action = accion;
  return accion;
}

/**
 * @brief Comportamiento del técnico para el Nivel 4.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoTecnico::ComportamientoTecnicoNivel_4(Sensores sensores) {
  return IDLE;
}

/**
 * @brief Comportamiento del técnico para el Nivel 5.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoTecnico::ComportamientoTecnicoNivel_5(Sensores sensores) {
  ActualizarMapa(sensores);
  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;

  switch (estado_t5) {
    case T5_ESPERANDO:
      if (sensores.venpaca) {
        dest_f_t5 = sensores.GotoF;
        dest_c_t5 = sensores.GotoC;
        obstaculos_t5.clear(); // Limpiamos la memoria para el nuevo tramo
        
        cout << "[TECNICO] Recibida llamada a (" << dest_f_t5 << ", " << dest_c_t5 << ")" << endl;
        
        EstadoT inicio;
        inicio.site.f = sensores.posF;
        inicio.site.c = sensores.posC;
        inicio.site.brujula = sensores.rumbo;
        inicio.zapatillas = tiene_zapatillas;

        EstadoT final;
        final.site.f = dest_f_t5;
        final.site.c = dest_c_t5;

        int limite_nodos = (sensores.nivel == 6) ? 2500 : -1;
        plan = AEstrellaTecnico(inicio, final, mapaResultado, mapaCotas, obstaculos_t5, limite_nodos);
        VisualizaPlan(inicio.site, plan);
        hayPlan = !plan.empty();
        estado_t5 = T5_IR_DESTINO;
      }
      return IDLE;

    case T5_IR_DESTINO:      
      // Sistema Anticolisiones
      if (hayPlan && !plan.empty()) {
        Action a = plan.front();
        if (a == WALK && sensores.agentes[2] != '_') {
            ubicacion del = Delante({sensores.posF, sensores.posC, sensores.rumbo});
            
            // 1. Si el obstáculo es el propio destino final, no lo vetamos. Esperamos.
            if (del.f == dest_f_t5 && del.c == dest_c_t5) {
                return IDLE; // Pausamos la ejecución del plan este tick
            } else {
                // Es un obstáculo intermedio, lo vetamos y recalculamos
                obstaculos_t5.insert({del.f, del.c});
                cout << "[TECNICO] ¡Bloqueo en (" << del.f << ", " << del.c << ")! Recalculando..." << endl;
                hayPlan = false;
                plan.clear();
            }
        }
      }

      // Si nos quedamos sin plan, recalculamos
      if (!hayPlan) {
        EstadoT inicio;
        inicio.site.f = sensores.posF;
        inicio.site.c = sensores.posC;
        inicio.site.brujula = sensores.rumbo;
        inicio.zapatillas = tiene_zapatillas;
        
        EstadoT final;
        final.site.f = dest_f_t5;
        final.site.c = dest_c_t5;
        
        int limite_nodos = (sensores.nivel == 6) ? 2500 : -1;
        plan = AEstrellaTecnico(inicio, final, mapaResultado, mapaCotas, obstaculos_t5, limite_nodos);
        VisualizaPlan(inicio.site, plan);
        hayPlan = !plan.empty();
        
        // 2. Si sigue sin haber plan (estamos acorralados), limpiamos obstáculos para el próximo ciclo
        if (!hayPlan) {
            obstaculos_t5.clear();
        }
      }

      if (hayPlan && !plan.empty()) {
        Action a = plan.front();
        plan.pop_front();
        if (plan.empty()) hayPlan = false; 
        return a;
      }

      if (sensores.posF == dest_f_t5 && sensores.posC == dest_c_t5) {
        hayPlan = false;
        plan.clear();
        estado_t5 = T5_MIRAR_INGENIERO;
      }
      else {
        return IDLE;
      }

    case T5_MIRAR_INGENIERO:
      if (sensores.enfrente) {
        cout << "[TECNICO] Sincronizando INSTALL." << endl;
        estado_t5 = T5_ESPERANDO; 
        return INSTALL;           
      } else {
        if (sensores.agentes[2] == 'i')
          return IDLE;
        else
          return TURN_SL;           
      }
  }

  return IDLE;
}

/**
 * @brief Comportamiento del técnico para el Nivel 6.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoTecnico::ComportamientoTecnicoNivel_6(Sensores sensores) {
  ActualizarMapa(sensores);
  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;

  int actual_conocido = 0;
  for(int i = 0; i < mapaResultado.size(); i++) {
      for(int j = 0; j < mapaResultado[0].size(); j++) {
          if(mapaResultado[i][j] != '?') actual_conocido++;
      }
  }

  // Solo se mueve si el Ingeniero lo llama o ya están en plena construcción
  if (sensores.venpaca || construyendo_t6) {
      if (!construyendo_t6) {
          construyendo_t6 = true;
          hayPlan = false; plan.clear();
          n5_stuck_t6 = false;
      }

      if (n5_stuck_t6) {
          if (actual_conocido > mapa_conocido_t6) {
              n5_stuck_t6 = false; 
          } else {
              return ExploracionGuiadaNivel6(sensores); 
          }
      }

      EstadoTecnicoN5 estado_previo = estado_t5;
      Action accion_n5 = ComportamientoTecnicoNivel_5(sensores);

      if (accion_n5 == IDLE && !hayPlan && estado_previo == T5_IR_DESTINO && estado_t5 == T5_IR_DESTINO) {
          cout << "[TECNICO N6] Camino bloqueado por niebla. Modo Sabueso Activado..." << endl;
          n5_stuck_t6 = true;
          mapa_conocido_t6 = actual_conocido;
          return ExploracionGuiadaNivel6(sensores);
      } 
      
      return accion_n5;
  }

  // MODO AHORRO DE BATERÍA: No gasta nada hasta que haya plan
  return IDLE;
}


// =========================================================================
// FUNCIONES PROPORCIONADAS
// =========================================================================

/**
 * @brief Actualiza el mapaResultado y mapaCotas con la información de los sensores.
 * @param sensores Datos actuales de los sensores.
 */
void ComportamientoTecnico::ActualizarMapa(Sensores sensores) {
  mapaResultado[sensores.posF][sensores.posC] = sensores.superficie[0];
  mapaCotas[sensores.posF][sensores.posC] = sensores.cota[0];

  int pos = 1;
  switch (sensores.rumbo) {
    case norte:
      for (int j = 1; j < 4; j++)
        for (int i = -j; i <= j; i++) {
          mapaResultado[sensores.posF - j][sensores.posC + i] = sensores.superficie[pos];
          mapaCotas[sensores.posF - j][sensores.posC + i] = sensores.cota[pos++];
        }
      break;
    case noreste:
      mapaResultado[sensores.posF - 1][sensores.posC] = sensores.superficie[1];
      mapaCotas[sensores.posF - 1][sensores.posC] = sensores.cota[1];
      mapaResultado[sensores.posF - 1][sensores.posC + 1] = sensores.superficie[2];
      mapaCotas[sensores.posF - 1][sensores.posC + 1] = sensores.cota[2];
      mapaResultado[sensores.posF][sensores.posC + 1] = sensores.superficie[3];
      mapaCotas[sensores.posF][sensores.posC + 1] = sensores.cota[3];
      mapaResultado[sensores.posF - 2][sensores.posC] = sensores.superficie[4];
      mapaCotas[sensores.posF - 2][sensores.posC] = sensores.cota[4];
      mapaResultado[sensores.posF - 2][sensores.posC + 1] = sensores.superficie[5];
      mapaCotas[sensores.posF - 2][sensores.posC + 1] = sensores.cota[5];
      mapaResultado[sensores.posF - 2][sensores.posC + 2] = sensores.superficie[6];
      mapaCotas[sensores.posF - 2][sensores.posC + 2] = sensores.cota[6];
      mapaResultado[sensores.posF - 1][sensores.posC + 2] = sensores.superficie[7];
      mapaCotas[sensores.posF - 1][sensores.posC + 2] = sensores.cota[7];
      mapaResultado[sensores.posF][sensores.posC + 2] = sensores.superficie[8];
      mapaCotas[sensores.posF][sensores.posC + 2] = sensores.cota[8];
      mapaResultado[sensores.posF - 3][sensores.posC] = sensores.superficie[9];
      mapaCotas[sensores.posF - 3][sensores.posC] = sensores.cota[9];
      mapaResultado[sensores.posF - 3][sensores.posC + 1] = sensores.superficie[10];
      mapaCotas[sensores.posF - 3][sensores.posC + 1] = sensores.cota[10];
      mapaResultado[sensores.posF - 3][sensores.posC + 2] = sensores.superficie[11];
      mapaCotas[sensores.posF - 3][sensores.posC + 2] = sensores.cota[11];
      mapaResultado[sensores.posF - 3][sensores.posC + 3] = sensores.superficie[12];
      mapaCotas[sensores.posF - 3][sensores.posC + 3] = sensores.cota[12];
      mapaResultado[sensores.posF - 2][sensores.posC + 3] = sensores.superficie[13];
      mapaCotas[sensores.posF - 2][sensores.posC + 3] = sensores.cota[13];
      mapaResultado[sensores.posF - 1][sensores.posC + 3] = sensores.superficie[14];
      mapaCotas[sensores.posF - 1][sensores.posC + 3] = sensores.cota[14];
      mapaResultado[sensores.posF][sensores.posC + 3] = sensores.superficie[15];
      mapaCotas[sensores.posF][sensores.posC + 3] = sensores.cota[15];
      break;
    case este:
      for (int j = 1; j < 4; j++)
        for (int i = -j; i <= j; i++) {
          mapaResultado[sensores.posF + i][sensores.posC + j] = sensores.superficie[pos];
          mapaCotas[sensores.posF + i][sensores.posC + j] = sensores.cota[pos++];
        }
      break;
    case sureste:
      mapaResultado[sensores.posF][sensores.posC + 1] = sensores.superficie[1];
      mapaCotas[sensores.posF][sensores.posC + 1] = sensores.cota[1];
      mapaResultado[sensores.posF + 1][sensores.posC + 1] = sensores.superficie[2];
      mapaCotas[sensores.posF + 1][sensores.posC + 1] = sensores.cota[2];
      mapaResultado[sensores.posF + 1][sensores.posC] = sensores.superficie[3];
      mapaCotas[sensores.posF + 1][sensores.posC] = sensores.cota[3];
      mapaResultado[sensores.posF][sensores.posC + 2] = sensores.superficie[4];
      mapaCotas[sensores.posF][sensores.posC + 2] = sensores.cota[4];
      mapaResultado[sensores.posF + 1][sensores.posC + 2] = sensores.superficie[5];
      mapaCotas[sensores.posF + 1][sensores.posC + 2] = sensores.cota[5];
      mapaResultado[sensores.posF + 2][sensores.posC + 2] = sensores.superficie[6];
      mapaCotas[sensores.posF + 2][sensores.posC + 2] = sensores.cota[6];
      mapaResultado[sensores.posF + 2][sensores.posC + 1] = sensores.superficie[7];
      mapaCotas[sensores.posF + 2][sensores.posC + 1] = sensores.cota[7];
      mapaResultado[sensores.posF + 2][sensores.posC] = sensores.superficie[8];
      mapaCotas[sensores.posF + 2][sensores.posC] = sensores.cota[8];
      mapaResultado[sensores.posF][sensores.posC + 3] = sensores.superficie[9];
      mapaCotas[sensores.posF][sensores.posC + 3] = sensores.cota[9];
      mapaResultado[sensores.posF + 1][sensores.posC + 3] = sensores.superficie[10];
      mapaCotas[sensores.posF + 1][sensores.posC + 3] = sensores.cota[10];
      mapaResultado[sensores.posF + 2][sensores.posC + 3] = sensores.superficie[11];
      mapaCotas[sensores.posF + 2][sensores.posC + 3] = sensores.cota[11];
      mapaResultado[sensores.posF + 3][sensores.posC + 3] = sensores.superficie[12];
      mapaCotas[sensores.posF + 3][sensores.posC + 3] = sensores.cota[12];
      mapaResultado[sensores.posF + 3][sensores.posC + 2] = sensores.superficie[13];
      mapaCotas[sensores.posF + 3][sensores.posC + 2] = sensores.cota[13];
      mapaResultado[sensores.posF + 3][sensores.posC + 1] = sensores.superficie[14];
      mapaCotas[sensores.posF + 3][sensores.posC + 1] = sensores.cota[14];
      mapaResultado[sensores.posF + 3][sensores.posC] = sensores.superficie[15];
      mapaCotas[sensores.posF + 3][sensores.posC] = sensores.cota[15];
      break;
    case sur:
      for (int j = 1; j < 4; j++)
        for (int i = -j; i <= j; i++) {
          mapaResultado[sensores.posF + j][sensores.posC - i] = sensores.superficie[pos];
          mapaCotas[sensores.posF + j][sensores.posC - i] = sensores.cota[pos++];
        }
      break;
    case suroeste:
      mapaResultado[sensores.posF + 1][sensores.posC] = sensores.superficie[1];
      mapaCotas[sensores.posF + 1][sensores.posC] = sensores.cota[1];
      mapaResultado[sensores.posF + 1][sensores.posC - 1] = sensores.superficie[2];
      mapaCotas[sensores.posF + 1][sensores.posC - 1] = sensores.cota[2];
      mapaResultado[sensores.posF][sensores.posC - 1] = sensores.superficie[3];
      mapaCotas[sensores.posF][sensores.posC - 1] = sensores.cota[3];
      mapaResultado[sensores.posF + 2][sensores.posC] = sensores.superficie[4];
      mapaCotas[sensores.posF + 2][sensores.posC] = sensores.cota[4];
      mapaResultado[sensores.posF + 2][sensores.posC - 1] = sensores.superficie[5];
      mapaCotas[sensores.posF + 2][sensores.posC - 1] = sensores.cota[5];
      mapaResultado[sensores.posF + 2][sensores.posC - 2] = sensores.superficie[6];
      mapaCotas[sensores.posF + 2][sensores.posC - 2] = sensores.cota[6];
      mapaResultado[sensores.posF + 1][sensores.posC - 2] = sensores.superficie[7];
      mapaCotas[sensores.posF + 1][sensores.posC - 2] = sensores.cota[7];
      mapaResultado[sensores.posF][sensores.posC - 2] = sensores.superficie[8];
      mapaCotas[sensores.posF][sensores.posC - 2] = sensores.cota[8];
      mapaResultado[sensores.posF + 3][sensores.posC] = sensores.superficie[9];
      mapaCotas[sensores.posF + 3][sensores.posC] = sensores.cota[9];
      mapaResultado[sensores.posF + 3][sensores.posC - 1] = sensores.superficie[10];
      mapaCotas[sensores.posF + 3][sensores.posC - 1] = sensores.cota[10];
      mapaResultado[sensores.posF + 3][sensores.posC - 2] = sensores.superficie[11];
      mapaCotas[sensores.posF + 3][sensores.posC - 2] = sensores.cota[11];
      mapaResultado[sensores.posF + 3][sensores.posC - 3] = sensores.superficie[12];
      mapaCotas[sensores.posF + 3][sensores.posC - 3] = sensores.cota[12];
      mapaResultado[sensores.posF + 2][sensores.posC - 3] = sensores.superficie[13];
      mapaCotas[sensores.posF + 2][sensores.posC - 3] = sensores.cota[13];
      mapaResultado[sensores.posF + 1][sensores.posC - 3] = sensores.superficie[14];
      mapaCotas[sensores.posF + 1][sensores.posC - 3] = sensores.cota[14];
      mapaResultado[sensores.posF][sensores.posC - 3] = sensores.superficie[15];
      mapaCotas[sensores.posF][sensores.posC - 3] = sensores.cota[15];
      break;
    case oeste:
      for (int j = 1; j < 4; j++)
        for (int i = -j; i <= j; i++) {
          mapaResultado[sensores.posF - i][sensores.posC - j] = sensores.superficie[pos];
          mapaCotas[sensores.posF - i][sensores.posC - j] = sensores.cota[pos++];
        }
      break;
    case noroeste:
      mapaResultado[sensores.posF][sensores.posC - 1] = sensores.superficie[1];
      mapaCotas[sensores.posF][sensores.posC - 1] = sensores.cota[1];
      mapaResultado[sensores.posF - 1][sensores.posC - 1] = sensores.superficie[2];
      mapaCotas[sensores.posF - 1][sensores.posC - 1] = sensores.cota[2];
      mapaResultado[sensores.posF - 1][sensores.posC] = sensores.superficie[3];
      mapaCotas[sensores.posF - 1][sensores.posC] = sensores.cota[3];
      mapaResultado[sensores.posF][sensores.posC - 2] = sensores.superficie[4];
      mapaCotas[sensores.posF][sensores.posC - 2] = sensores.cota[4];
      mapaResultado[sensores.posF - 1][sensores.posC - 2] = sensores.superficie[5];
      mapaCotas[sensores.posF - 1][sensores.posC - 2] = sensores.cota[5];
      mapaResultado[sensores.posF - 2][sensores.posC - 2] = sensores.superficie[6];
      mapaCotas[sensores.posF - 2][sensores.posC - 2] = sensores.cota[6];
      mapaResultado[sensores.posF - 2][sensores.posC - 1] = sensores.superficie[7];
      mapaCotas[sensores.posF - 2][sensores.posC - 1] = sensores.cota[7];
      mapaResultado[sensores.posF - 2][sensores.posC] = sensores.superficie[8];
      mapaCotas[sensores.posF - 2][sensores.posC] = sensores.cota[8];
      mapaResultado[sensores.posF][sensores.posC - 3] = sensores.superficie[9];
      mapaCotas[sensores.posF][sensores.posC - 3] = sensores.cota[9];
      mapaResultado[sensores.posF - 1][sensores.posC - 3] = sensores.superficie[10];
      mapaCotas[sensores.posF - 1][sensores.posC - 3] = sensores.cota[10];
      mapaResultado[sensores.posF - 2][sensores.posC - 3] = sensores.superficie[11];
      mapaCotas[sensores.posF - 2][sensores.posC - 3] = sensores.cota[11];
      mapaResultado[sensores.posF - 3][sensores.posC - 3] = sensores.superficie[12];
      mapaCotas[sensores.posF - 3][sensores.posC - 3] = sensores.cota[12];
      mapaResultado[sensores.posF - 3][sensores.posC - 2] = sensores.superficie[13];
      mapaCotas[sensores.posF - 3][sensores.posC - 2] = sensores.cota[13];
      mapaResultado[sensores.posF - 3][sensores.posC - 1] = sensores.superficie[14];
      mapaCotas[sensores.posF - 3][sensores.posC - 1] = sensores.cota[14];
      mapaResultado[sensores.posF - 3][sensores.posC] = sensores.superficie[15];
      mapaCotas[sensores.posF - 3][sensores.posC] = sensores.cota[15];
      break;
  }
}



/**
 * @brief Determina si una casilla es transitable para el técnico.
 * En esta práctica, si el técnico tiene zapatillas, el bosque ('B') es transitable.
 * @param f Fila de la casilla.
 * @param c Columna de la casilla.
 * @param tieneZapatillas Indica si el agente posee las zapatillas.
 * @return true si la casilla es transitable.
 */
bool ComportamientoTecnico::EsCasillaTransitableLevel0(int f, int c, bool tieneZapatillas) {
  if (f < 0 || f >= mapaResultado.size() || c < 0 || c >= mapaResultado[0].size()) return false;
  return es_camino(mapaResultado[f][c]);  // Solo 'C', 'S', 'D', 'U' son transitables en Nivel 0
}

bool ComportamientoTecnico::EsCasillaTransitable(int f, int c, bool tieneZapatillas) {
  if (f < 0 || f >= mapaResultado.size() || c < 0 || c >= mapaResultado[0].size()) return false;

  unsigned char sup = mapaResultado[f][c];

  // 1. Muros y Precipicios siempre bloquean
  if (sup == 'P' || sup == 'M') return false;

  // 2. El Bosque bloquea SI NO tienes las zapatillas
  if (sup == 'B' && !tieneZapatillas) return false;

  // 3. Si no es un muro, no es un precipicio, y no es un bosque sin zapatillas, es transitable
  return true; 
}

/**
 * @brief Comprueba si la casilla de delante es accesible por diferencia de altura.
 * Para el técnico: desnivel máximo siempre 1.
 * @param actual Estado actual del agente (fila, columna, orientacion).
 * @return true si el desnivel con la casilla de delante es admisible.
 */
bool ComportamientoTecnico::EsAccesiblePorAltura(const ubicacion &actual) {
  ubicacion del = Delante(actual);
  if (del.f < 0 || del.f >= mapaCotas.size() || del.c < 0 || del.c >= mapaCotas[0].size()) return false;
  int desnivel = abs(mapaCotas[del.f][del.c] - mapaCotas[actual.f][actual.c]);
  if (desnivel > 1) return false;
  return true;
}

/**
 * @brief Devuelve la posición (fila, columna) de la casilla que hay delante del agente.
 * Calcula la casilla frontal según la orientación actual (8 direcciones).
 * @param actual Estado actual del agente (fila, columna, orientacion).
 * @return Estado con la fila y columna de la casilla de enfrente.
 */
ubicacion ComportamientoTecnico::Delante(const ubicacion &actual) const {
  ubicacion delante = actual;
  switch (actual.brujula) {
    case 0: delante.f--; break;                        // norte
    case 1: delante.f--; delante.c++; break;     // noreste
    case 2: delante.c++; break;                     // este
    case 3: delante.f++; delante.c++; break;     // sureste
    case 4: delante.f++; break;                        // sur
    case 5: delante.f++; delante.c--; break;     // suroeste
    case 6: delante.c--; break;                     // oeste
    case 7: delante.f--; delante.c--; break;     // noroeste
  }
  return delante;
}


/**
 * @brief Imprime por consola la secuencia de acciones de un plan.
 *
 * @param plan  Lista de acciones del plan.
 */
void ComportamientoTecnico::PintaPlan(const list<Action> &plan)
{
  auto it = plan.begin();
  while (it != plan.end())
  {
    if (*it == WALK)
    {
      cout << "W ";
    }
    else if (*it == JUMP)
    {
      cout << "J ";
    }
    else if (*it == TURN_SR)
    {
      cout << "r ";
    }
    else if (*it == TURN_SL)
    {
      cout << "l ";
    }
    else if (*it == COME)
    {
      cout << "C ";
    }
    else if (*it == IDLE)
    {
      cout << "I ";
    }
    else
    {
      cout << "-_ ";
    }
    it++;
  }
  cout << "( longitud " << plan.size() << ")" << endl;
}



/**
 * @brief Convierte un plan de acciones en una lista de casillas para
 *        su visualización en el mapa 2D.
 *
 * @param st    Estado de partida.
 * @param plan  Lista de acciones del plan.
 */
void ComportamientoTecnico::VisualizaPlan(const ubicacion &st,
                                            const list<Action> &plan)
{
   listaPlanCasillas.clear();
  ubicacion cst = st;

  listaPlanCasillas.push_back({cst.f, cst.c, WALK});
  auto it = plan.begin();
  while (it != plan.end())
  {

    switch (*it)
    {
    case JUMP:
      switch (cst.brujula)
      {
      case 0:
        cst.f--;
        break;
      case 1:
        cst.f--;
        cst.c++;
        break;
      case 2:
        cst.c++;
        break;
      case 3:
        cst.f++;
        cst.c++;
        break;
      case 4:
        cst.f++;
        break;
      case 5:
        cst.f++;
        cst.c--;
        break;
      case 6:
        cst.c--;
        break;
      case 7:
        cst.f--;
        cst.c--;
        break;
      }
      if (cst.f >= 0 && cst.f < mapaResultado.size() &&
          cst.c >= 0 && cst.c < mapaResultado[0].size())
        listaPlanCasillas.push_back({cst.f, cst.c, JUMP});
    case WALK:
      switch (cst.brujula)
      {
      case 0:
        cst.f--;
        break;
      case 1:
        cst.f--;
        cst.c++;
        break;
      case 2:
        cst.c++;
        break;
      case 3:
        cst.f++;
        cst.c++;
        break;
      case 4:
        cst.f++;
        break;
      case 5:
        cst.f++;
        cst.c--;
        break;
      case 6:
        cst.c--;
        break;
      case 7:
        cst.f--;
        cst.c--;
        break;
      }
      if (cst.f >= 0 && cst.f < mapaResultado.size() &&
          cst.c >= 0 && cst.c < mapaResultado[0].size())
        listaPlanCasillas.push_back({cst.f, cst.c, WALK});
      break;
    case TURN_SR:
      cst.brujula = (Orientacion) (( (int) cst.brujula + 1) % 8);
      break;
    case TURN_SL:
      cst.brujula = (Orientacion) (( (int) cst.brujula + 7) % 8);
      break;
    }
    it++;
  }
}

// =========================================================================
// FUNCIONES AUXILIARES DEL TÉCNICO
// =========================================================================
/**
 * @brief Comprueba si el movimiento a la casilla de delante es viable, según sea esta transitable y la altura que tenga
 * @param actual    Casilla actual del agente
 * @param zap  Booleano que es true si tiene las zapatillas
 * @return True si la casilla es transitable y tiene un desnivel aceptable
 */
bool ComportamientoTecnico::AndarViable0(const ubicacion &actual, bool zap) {
  bool viable;

  viable = EsCasillaTransitableLevel0(Delante(actual).f, Delante(actual).c, zap) && EsAccesiblePorAltura(actual);

  return viable;
}

bool ComportamientoTecnico::AndarViable(const ubicacion &actual, bool zap) {
  bool viable;

  viable = EsCasillaTransitable(Delante(actual).f, Delante(actual).c, zap) && EsAccesiblePorAltura(actual);

  return viable;
}

int ComportamientoTecnico::costeEnergiaT(Action accion, const EstadoT &st, 
                                         const vector<vector<unsigned char>> &terreno, 
                                         const vector<vector<unsigned char>> &altura) {
  int coste = 0;
  unsigned char sup_actual = terreno[st.site.f][st.site.c]; 
  
  switch(accion) {
    case WALK:
    {
      // 1. Coste base según la casilla de INICIO
      if (sup_actual == 'A') coste = 60;
      else if (sup_actual == 'H') coste = 6;
      else if (sup_actual == 'S') coste = 3;
      else coste = 1; // Para 'C', 'X', 'U', 'D', 'B' (con zapatillas)...
      
      // 2. Modificador por altura (SOLO aplica si sales de A, H o S)
      if (sup_actual == 'A' || sup_actual == 'H' || sup_actual == 'S') {
          ubicacion del = Delante(st.site);
          int alt_actual = altura[st.site.f][st.site.c];
          int alt_del = altura[del.f][del.c];
          
          if (alt_del > alt_actual) coste += 5;       // Subir
          else if (alt_del < alt_actual) coste -= 2;  // Bajar
      }
      break;
    }
        
    case TURN_SR:
    case TURN_SL:
    {
      // Gasto normal de giro según casilla de INICIO
      if (sup_actual == 'A') coste = 5;
      else if (sup_actual == 'H') coste = 2;
      else if (sup_actual == 'S') coste = 1;
      else coste = 1; 
      break;
    }
  }
  return coste;
}

EstadoT ComportamientoTecnico::applyT(Action accion, const EstadoT &st, 
                                      const vector<vector<unsigned char>> &terreno, 
                                      const vector<vector<unsigned char>> &altura,
                                      const set<pair<int, int>> &obstaculos) {
  EstadoT next = st;
  switch (accion) {
    case WALK:
      if (AndarViable(st.site, st.zapatillas)) {
        ubicacion del = Delante(st.site);
        // Si la casilla está bloqueada por el ingeniero, el movimiento es inválido
        if (obstaculos.find({del.f, del.c}) != obstaculos.end()) {
            return st; // Devolver el estado original significa fallo en applyT
        }

        if (terreno[del.f][del.c] == '?') {
            return st;
        }
        
        next.site = del;
        if (terreno[next.site.f][next.site.c] == 'D') {
          next.zapatillas = true;
        }
      }
      break;
    case TURN_SR:
      next.site.brujula = static_cast<Orientacion>((next.site.brujula + 1) % 8);
      break;
    case TURN_SL:
      next.site.brujula = static_cast<Orientacion>((next.site.brujula + 7) % 8);
      break;
  }
  return next;
}

std::list<Action> ComportamientoTecnico::AEstrellaTecnico(const EstadoT &inicio, const EstadoT &final, 
                                     const std::vector<std::vector<unsigned char>> &terreno, 
                                     const std::vector<std::vector<unsigned char>> &altura,
                                     const std::set<std::pair<int, int>> &obstaculos,
                                     int limite_nodos) {
  priority_queue<NodoT, vector<NodoT>, ComparaNodosT> frontier;
  map<EstadoT, int> explored_costs; 
  list<Action> path;

  NodoT current_node;
  current_node.estado = inicio;
  current_node.coste_g = 0;
  // Heurística de Chebyshev
  current_node.heuristica_h = max(abs(inicio.site.f - final.site.f), abs(inicio.site.c - final.site.c));
  
  frontier.push(current_node);
  explored_costs[inicio] = 0;
  
  bool SolutionFound = false;
  int  nodos_expandidos = 0;

  while (!frontier.empty() && !SolutionFound) {
    nodos_expandidos++;
    if (limite_nodos > 0 && nodos_expandidos > limite_nodos) {
        break; // <--- Salvavidas de CPU exclusivo para Nivel 6
    }

    current_node = frontier.top();
    frontier.pop();

    // ¿Llegamos a la meta?
    if (current_node.estado.site.f == final.site.f && current_node.estado.site.c == final.site.c) {
      SolutionFound = true;
      path = current_node.secuencia;
      break;
    }

    // Si hemos encontrado una ruta mejor hacia este nodo desde que lo metimos en la cola, lo saltamos
    if (explored_costs[current_node.estado] < current_node.coste_g) {
      continue;
    }

    Action acciones[] = {WALK, TURN_SR, TURN_SL};
    
    for (Action accion : acciones) {
      NodoT child;
      child.estado = applyT(accion, current_node.estado, terreno, altura, obstaculos);
      
      // Movimiento inválido (chocó, desnivel mayor que 1, etc.)
      if (child.estado == current_node.estado) continue;

      int coste_accion = costeEnergiaT(accion, current_node.estado, terreno, altura);
      child.coste_g = current_node.coste_g + coste_accion;
      child.heuristica_h = max(abs(child.estado.site.f - final.site.f), abs(child.estado.site.c - final.site.c));
      
      child.secuencia = current_node.secuencia;
      child.secuencia.push_back(accion);

      // Si es la primera vez que llegamos a este estado, o lo hacemos con MENOR coste de energía
      if (explored_costs.find(child.estado) == explored_costs.end() || 
        child.coste_g < explored_costs[child.estado]) {
        
        explored_costs[child.estado] = child.coste_g;
        frontier.push(child);
      }
    }
  }

  return path;
}

/**
 * @brief Exploración específica para el Técnico en el Nivel 6.
 * Si el A* no encuentra camino al Ingeniero por culpa de la niebla, 
 * el técnico explora guiado magnéticamente hacia las coordenadas de destino.
 */
/**
 * @brief Exploración específica para el Técnico en el Nivel 6.
 * "Modo Rompehielos": Si el A* falla por niebla, el Técnico actúa como un misil 
 * hacia las coordenadas del Ingeniero, priorizando atravesar la niebla ('?') 
 * en líneas rectas (alta inercia) para despejar el camino rápido.
 */
Action ComportamientoTecnico::ExploracionGuiadaNivel6(Sensores sensores) {
  Action accion = IDLE;

  ActualizarMapa(sensores);

  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;

  mapa_visitas[{sensores.posF, sensores.posC}]++;

  int prioridades[3];
  ubicacion ubicaciones[3];
  ubicacion actual = {sensores.posF, sensores.posC, sensores.rumbo};
  Orientacion izq = static_cast<Orientacion>((sensores.rumbo + 7) % 8);
  Orientacion der = static_cast<Orientacion>((sensores.rumbo + 1) % 8);
  ubicacion mira_izq = {sensores.posF, sensores.posC, izq};
  ubicacion mira_der = {sensores.posF, sensores.posC, der};

  ubicaciones[0] = mira_izq;
  ubicaciones[1] = actual;
  ubicaciones[2] = mira_der;

  // --- BUCLE DE ANDAR ---
  for (int i = 0; i < 3; i++) {
    ubicacion destino = Delante(ubicaciones[i]);
    unsigned char sup = sensores.superficie[i+1];

    if (sensores.agentes[i+1] != '_') {
      mapa_visitas[{destino.f, destino.c}] += 5;
    }

    int visitas = mapa_visitas[{destino.f, destino.c}];

    if (!AndarViable(ubicaciones[i], tiene_zapatillas) || sensores.agentes[i+1] != '_') {
      prioridades[i] = 999999;
    } 
    else {
      int base_prioridad = visitas * 10000;
      
      if (sup == 'D' && !tiene_zapatillas) {
        prioridades[i] = base_prioridad - 50000;
      } 
      else if (sup == 'C' || sup == 'U') {
        prioridades[i] = base_prioridad - 30000;
      }
      else if (sup == 'S') {
        prioridades[i] = base_prioridad - 20000;
      }
      else if (sup == 'A') {
        prioridades[i] = base_prioridad + 20000;
      }
      else {
        // Hierba o Zapatillas ya conseguidas
        prioridades[i] = base_prioridad;
      }

      // 1. Si el ingeniero le ha llamado, eso es lo MÁS importante
      if (estado_t5 == T5_IR_DESTINO && dest_f_t5 != -1) {
        if (abs(destino.f - dest_f_t5) + abs(destino.c - dest_c_t5) < abs(sensores.posF - dest_f_t5) + abs(sensores.posC - dest_c_t5)) {
          prioridades[i] -= 45000; // Prioridad máxima absoluta
        }
      } 
      // 2. Si está libre, usa el GPS para ir a la Belkanita por su cuenta (ayuda a mapear)
      else if (sensores.BelPosF != -1) {
        if (abs(destino.f - sensores.BelPosF) + abs(destino.c - sensores.BelPosC) < abs(sensores.posF - sensores.BelPosF) + abs(sensores.posC - sensores.BelPosC)) {
          prioridades[i] -= 25000;
        }
      }
    }
  }

  int inercia = (sensores.nivel == 6) ? 6000 : 1;

  if (prioridades[1] < 999999) prioridades[1] -= inercia; // WALK

  prioridades[2] -= 2;

  int min_frente = *min_element(prioridades, prioridades + 3); 

  if (min_frente >= 0) {
    int mejor_prio_radar = min_frente; 
    int mejor_giro = -1; 

    for (int g = 1; g < 8; g++) {
      Orientacion ori = static_cast<Orientacion>((sensores.rumbo + g) % 8);
      ubicacion mira = {sensores.posF, sensores.posC, ori};

      if (AndarViable(mira, tiene_zapatillas)) {
        ubicacion dest_radar = Delante(mira);
        unsigned char sup = mapaResultado[dest_radar.f][dest_radar.c];
        
        int prio_lateral = mapa_visitas[{dest_radar.f, dest_radar.c}] * 10000;
        
        if (sup == 'D' && !tiene_zapatillas) prio_lateral -= 50000;
        else if (sup == 'C' || sup == 'U') prio_lateral -= 30000;
        else if (sup == 'S') prio_lateral -= 20000;
        else if (sup == 'A') prio_lateral += 20000;

        if (prio_lateral < mejor_prio_radar) {
          mejor_prio_radar = prio_lateral;
          if (g >= 1 && g <= 3) mejor_giro = 2; 
          else mejor_giro = 0; 
        }
      }
    }

    if (mejor_giro != -1) {
      prioridades[mejor_giro] = -1000000; 
    }
  }

  int min = distance(prioridades, min_element(prioridades, prioridades + 3));

  // Si acabamos de girar a un lado, prohibimos girar al lado contrario inmediatamente
  // para romper cualquier bucle de Izquierda-Derecha.
  if (min == 0 && last_action == TURN_SR) {
    prioridades[0] = 999999;
    min = distance(prioridades, min_element(prioridades, prioridades + 3));
  } else if (min == 2 && last_action == TURN_SL) {
    prioridades[2] = 999999;
    min = distance(prioridades, min_element(prioridades, prioridades + 3));
  }

  switch (min) {
  case 0:
    accion = TURN_SL;
    break;
  case 1:
    accion = WALK;
    break;
  case 2:
    accion = TURN_SR;
    break;
  }

  last_action = accion;
  return accion;
}