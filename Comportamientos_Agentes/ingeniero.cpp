#include "ingeniero.hpp"
#include "motorlib/util.h"
#include <iostream>
#include <queue>
#include <set>
#include <algorithm>

using namespace std;

// =========================================================================
// ÁREA DE IMPLEMENTACIÓN DEL ESTUDIANTE
// =========================================================================

Action ComportamientoIngeniero::think(Sensores sensores)
{
  Action accion = IDLE;

  // Decisión del agente según el nivel
  switch (sensores.nivel)
  {
  case 0:
    accion = ComportamientoIngenieroNivel_0(sensores);
    break;
  case 1:
    accion = ComportamientoIngenieroNivel_1(sensores);
    break;
  case 2:
    accion = ComportamientoIngenieroNivel_2(sensores);
    break;
  case 3:
    accion = ComportamientoIngenieroNivel_3(sensores);
    break;
  case 4:
    accion = ComportamientoIngenieroNivel_4(sensores);
    break;
  case 5:
    accion = ComportamientoIngenieroNivel_5(sensores);
    break;
  case 6:
    accion = ComportamientoIngenieroNivel_6(sensores);
    break;
  }

  return accion;
}

// Niveles iniciales (Comportamientos reactivos simples)
Action ComportamientoIngeniero::ComportamientoIngenieroNivel_0(Sensores sensores) {
  Action accion = IDLE;

  ActualizarMapa(sensores);

  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;
  if (sensores.superficie[0] == 'U') return IDLE;

  mapa_visitas[{sensores.posF, sensores.posC}]++;

  int prioridades[6];
  ubicacion ubicaciones[6];
  ubicacion actual = {sensores.posF, sensores.posC, sensores.rumbo};
  Orientacion izq = static_cast<Orientacion>((sensores.rumbo + 7) % 8);
  Orientacion der = static_cast<Orientacion>((sensores.rumbo + 1) % 8);
  ubicacion mira_izq = {sensores.posF, sensores.posC, izq};
  ubicacion mira_der = {sensores.posF, sensores.posC, der};

  ubicaciones[0] = mira_izq;
  ubicaciones[1] = actual;
  ubicaciones[2] = mira_der;
  ubicaciones[3] = mira_izq;
  ubicaciones[4] = actual;
  ubicaciones[5] = mira_der;

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

  for (int i = 3; i < 6; i++) {
    ubicacion destino = Delante(Delante(ubicaciones[i]));
    unsigned char sup = sensores.superficie[2*i - 2];

    if (sensores.agentes[2*i - 2] != '_') {
        mapa_visitas[{destino.f, destino.c}] = mapa_visitas[{destino.f, destino.c}] + 5;
    }

    if (SaltarViable0(ubicaciones[i], tiene_zapatillas) && sup == 'U' && sensores.agentes[2*i-2] == '_')
      prioridades[i] = -999999 + mapa_visitas[{destino.f, destino.c}];
    else if (SaltarViable0(ubicaciones[i], tiene_zapatillas) && (sup == 'D' && !tiene_zapatillas) && sensores.agentes[2*i-2] == '_')
      prioridades[i] = -500000 + mapa_visitas[{destino.f, destino.c}];
    else if (SaltarViable0(ubicaciones[i], tiene_zapatillas) && (sup == 'C' || (sup == 'D' && tiene_zapatillas)) && sensores.agentes[2*i-2] == '_')
      prioridades[i] = mapa_visitas[{destino.f, destino.c}];
    else
      prioridades[i] = 999999;
  }

  int min_frente = *min_element(prioridades, prioridades + 6); 

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

  int min = distance(prioridades, min_element(prioridades, prioridades + 6));

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
  case 3:
    accion = TURN_SL;
    break;
  case 4:
    accion = JUMP;
    break;
  case 5:
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
bool ComportamientoIngeniero::es_camino(unsigned char c) const
{
  return (c == 'C' || c == 'D' || c == 'U');
}

/**
 * @brief Comportamiento reactivo del ingeniero para el Nivel 1.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoIngeniero::ComportamientoIngenieroNivel_1(Sensores sensores)
{
  Action accion = IDLE;

  ActualizarMapa(sensores);

  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;

  mapa_visitas[{sensores.posF, sensores.posC}]++;

  int prioridades[6];
  ubicacion ubicaciones[6];
  ubicacion actual = {sensores.posF, sensores.posC, sensores.rumbo};
  Orientacion izq = static_cast<Orientacion>((sensores.rumbo + 7) % 8);
  Orientacion der = static_cast<Orientacion>((sensores.rumbo + 1) % 8);
  ubicacion mira_izq = {sensores.posF, sensores.posC, izq};
  ubicacion mira_der = {sensores.posF, sensores.posC, der};

  ubicaciones[0] = mira_izq;
  ubicaciones[1] = actual;
  ubicaciones[2] = mira_der;
  ubicaciones[3] = mira_izq;
  ubicaciones[4] = actual;
  ubicaciones[5] = mira_der;

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
      int base_prioridad = visitas * 10000; // Las visitas dominan la decisión
      
      if (sup == 'D' && !tiene_zapatillas) {
        prioridades[i] = base_prioridad - 50000; // Queremos las zapatillas
      } 
      else if (sup == 'C') {
        prioridades[i] = base_prioridad - 30000;   // Preferimos camino a hierba
      }
      else if (sup == 'S') {
        prioridades[i] = base_prioridad - 20000;
      }
      else if (sup == 'A') {
        prioridades[i] = base_prioridad + 20000; // Penalizamos mucho el agua
      } 
      else {
        prioridades[i] = base_prioridad;        // Hierba, o Zapatillas si ya las tenemos
      }
    }
  }

  // --- BUCLE DE SALTAR ---
  for (int i = 3; i < 6; i++) {
    ubicacion destino = Delante(Delante(ubicaciones[i]));
    unsigned char sup = sensores.superficie[2*i-2];

    if (sensores.agentes[2*i-2] != '_') {
      mapa_visitas[{destino.f, destino.c}] += 5; 
    }

    int visitas = mapa_visitas[{destino.f, destino.c}];

    if (!SaltarViable(ubicaciones[i], tiene_zapatillas) || sensores.agentes[2*i-2] != '_') {
      prioridades[i] = 999999;
    } 
    else {
      int base_prioridad = visitas * 10000;
      
      if (sup == 'D' && !tiene_zapatillas) {
        prioridades[i] = base_prioridad - 55000;
      } 
      else if (sup == 'C') {
        prioridades[i] = base_prioridad - 35000;
      }
      else if (sup == 'S') {
        prioridades[i] = base_prioridad - 25000;
      }
      else if (sup == 'A') {
        prioridades[i] = base_prioridad + 15000;
      } 
      else {
        prioridades[i] = base_prioridad;
      }
    }
  }

  int inercia = (sensores.nivel == 6) ? 15000 : 1;

  if (prioridades[1] < 999999) prioridades[1] -= inercia; // WALK
  if (prioridades[4] < 999999) prioridades[4] -= inercia; // JUMP

  int min_frente = *min_element(prioridades, prioridades + 6);

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

        // Si el lateral es ESTRICTAMENTE MEJOR que lo que tenemos delante
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

  int min = distance(prioridades, min_element(prioridades, prioridades + 6));

  // Si acabamos de girar a un lado, prohibimos girar al lado contrario inmediatamente
  // para romper cualquier bucle de Izquierda-Derecha.
  if (min == 0 && last_action == TURN_SR) {
    prioridades[0] = 999999;
    min = distance(prioridades, min_element(prioridades, prioridades + 6));
  } else if (min == 2 && last_action == TURN_SL) {
    prioridades[2] = 999999;
    min = distance(prioridades, min_element(prioridades, prioridades + 6));
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
  case 3:
    accion = TURN_SL;
    break;
  case 4:
    accion = JUMP;
    break;
  case 5:
    accion = TURN_SR;
    break;
  }

  last_action = accion;
  return accion;
}

// Niveles avanzados (Uso de búsqueda)
/**
 * @brief Comportamiento del ingeniero para el Nivel 2 (búsqueda).
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoIngeniero::ComportamientoIngenieroNivel_2(Sensores sensores)
{
  Action accion = IDLE;

    // Si no tenemos plan, lo calculamos
    if (!hayPlan) {
        EstadoI inicio;
        inicio.site.f = sensores.posF;
        inicio.site.c = sensores.posC;
        inicio.site.brujula = sensores.rumbo;
        inicio.zapatillas = tiene_zapatillas; // Mantener estado de las zapatillas si las cogió

        EstadoI final;
        final.site.f = sensores.BelPosF;
        final.site.c = sensores.BelPosC;

        plan = B_AnchuraIngeniero(inicio, final);
        VisualizaPlan(inicio.site, plan);

        hayPlan = !plan.empty();
    }

    // Si tenemos plan, ejecutamos la siguiente acción
    if (hayPlan && !plan.empty()) {
        accion = plan.front();
        plan.pop_front();
        
        // Si acabamos de ejecutar la última, reseteamos para no hacer cosas raras
        if (plan.empty()) {
            hayPlan = false;
        }
    }

    last_action = accion;
    return accion;
}

/**
 * @brief Comportamiento del ingeniero para el Nivel 3.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoIngeniero::ComportamientoIngenieroNivel_3(Sensores sensores)
{
  // Estrategia de Evasión Táctica:
  // Me quedo quieto (IDLE) para no estorbar, a menos que detecte al Técnico 
  // en mis narices (casillas 1, 2 o 3 del sensor frontal).
  
  bool tecnicoCerca = false;
  for (int i = 1; i <= 3; i++) {
    if (sensores.agentes[i] == 't') {
        tecnicoCerca = true;
        break;
    }
  }

  // Si el Técnico viene hacia mí, me quito de en medio
  if (tecnicoCerca) {
    ubicacion actual = {sensores.posF, sensores.posC, sensores.rumbo};
    if (AndarViable(actual, tiene_zapatillas)) {
        return WALK;   // Doy un paso adelante para apartarme
    } else {
        return TURN_SR; // Si hay pared, giro para poder apartarme en el siguiente ciclo
    }
  }

  // Si el Técnico no está cerca, no me muevo para no cruzarme por accidente
  return IDLE;
}

/**
 * @brief Comportamiento del ingeniero para el Nivel 4.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoIngeniero::ComportamientoIngenieroNivel_4(Sensores sensores)
{
  if (!hayPlan) {
    // Usamos DIRECTAMENTE el máximo ecológico para evitar problemas de variables sucias
    plan_tuberias = AEstrellaTuberias(sensores.BelPosF, sensores.BelPosC, 
                                      sensores.max_ecologico, 
                                      mapaResultado, mapaCotas);

    if (!plan_tuberias.empty()) {
      VisualizaRedTuberias(plan_tuberias);
      hayPlan = true;
    } else {
      cout << "El A* no encontró NINGÚN camino que respete el límite ecológico." << endl;
    }
  }
  return IDLE;
}

/**
 * @brief Comportamiento del ingeniero para el Nivel 5.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoIngeniero::ComportamientoIngenieroNivel_5(Sensores sensores) {
  ActualizarMapa(sensores);
  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;

  switch (estado_i5) {
    case I5_CALCULANDO_PLAN:
      if (plan_tuberias.empty()) {
        plan_tuberias = AEstrellaTuberias(sensores.BelPosF, sensores.BelPosC, 
                                          sensores.max_ecologico, 
                                          mapaResultado, mapaCotas);
        VisualizaRedTuberias(plan_tuberias);
      }
      
      if (!plan_tuberias.empty()) {
        cout << "[INGENIERO] Plan de tuberias OK. Tramos: " << plan_tuberias.size() << endl;
        paso_actual_i5 = plan_tuberias.begin();
        A_paso = *paso_actual_i5;
        
        auto next_it = paso_actual_i5;
        next_it++;
        if (next_it != plan_tuberias.end()) {
            B_paso = *next_it;
            estado_i5 = I5_IR_A;
        } else {
            estado_i5 = I5_COMPLETADO; 
        }
      } else {
        // Si no paran de salir estos mensajes, es que el A* no halla camino ecológico válido
        cout << "[INGENIERO] Calculando plan... A* fallo. Reintentando." << endl;
      }
      return IDLE;

    case I5_IR_A:
      if (sensores.posF == A_paso.fil && sensores.posC == A_paso.col) {
        hayPlan = false;
        plan.clear();
        estado_i5 = I5_ACONDICIONAR_A;
        return IDLE;
      }
      if (!hayPlan) {
        EstadoI inicio = {{sensores.posF, sensores.posC, sensores.rumbo}, tiene_zapatillas};
        EstadoI final = {{A_paso.fil, A_paso.col, norte}, false};
        plan = B_AnchuraIngeniero(inicio, final);
        VisualizaPlan(inicio.site, plan);
        hayPlan = !plan.empty();
      }
      if (hayPlan && !plan.empty()) {
        if (plan.front() == WALK && sensores.agentes[2] != '_') {
            return IDLE; 
        }
        Action a = plan.front(); plan.pop_front(); 
        if (plan.empty()) hayPlan = false;
        return a;
      }
      return IDLE;

    case I5_ACONDICIONAR_A:
      estado_i5 = I5_LLAMAR_TECNICO;
      if (A_paso.op == 1) return RAISE;
      if (A_paso.op == -1) return DIG;
      return IDLE;

    case I5_LLAMAR_TECNICO:
      cout << "[INGENIERO] Llamando al Tecnico a casilla (" << A_paso.fil << ", " << A_paso.col << ")" << endl;
      estado_i5 = I5_IR_B;
      return COME;

    case I5_IR_B:
      if (sensores.posF == B_paso.fil && sensores.posC == B_paso.col) {
        hayPlan = false;
        plan.clear();
        estado_i5 = I5_ACONDICIONAR_B;
        return IDLE;
      }
      if (!hayPlan) {
        EstadoI inicio = {{sensores.posF, sensores.posC, sensores.rumbo}, tiene_zapatillas};
        EstadoI final = {{B_paso.fil, B_paso.col, norte}, false};
        plan = B_AnchuraIngeniero(inicio, final);
        hayPlan = !plan.empty();
      }
      if (hayPlan && !plan.empty()) {
        if (plan.front() == WALK && sensores.agentes[2] != '_') {
            return IDLE; 
        }
        Action a = plan.front(); plan.pop_front(); 
        if (plan.empty()) hayPlan = false; // <--- CORRECCIÓN CLAVE
        return a;
      }
      return IDLE;

    case I5_ACONDICIONAR_B:
      estado_i5 = I5_MIRAR_A;
      if (B_paso.op == 1) return RAISE;
      if (B_paso.op == -1) return DIG;
      return IDLE;

    case I5_MIRAR_A:
    {
      int df = A_paso.fil - sensores.posF;
      int dc = A_paso.col - sensores.posC;
      Orientacion obj = sensores.rumbo;

      if (df == -1) obj = norte;
      else if (df == 1) obj = sur;
      else if (dc == -1) obj = oeste;
      else if (dc == 1) obj = este;

      // Si aún no miramos al objetivo, giramos
      if (sensores.rumbo != obj) {
        int diff = (obj - sensores.rumbo + 8) % 8;
        if (diff <= 4) return TURN_SR;
        else return TURN_SL;
      }
      
      // Si ya estamos mirando, evaluamos la sincro en este MISMO tick, sin esperar
      estado_i5 = I5_ESPERAR_SINCRO;
      if (sensores.enfrente) {
        cout << "[INGENIERO] Cara a cara! Sincronizando INSTALL." << endl;
        paso_actual_i5++;
        A_paso = *paso_actual_i5;
        
        auto next_it = paso_actual_i5;
        next_it++;
        
        if (next_it != plan_tuberias.end()) {
          B_paso = *next_it;
          estado_i5 = I5_LLAMAR_TECNICO; 
        } else {
          cout << "[INGENIERO] ¡Tubería completada!" << endl;
          estado_i5 = I5_COMPLETADO;
        }
        return INSTALL;
      }
      return IDLE;
    }

    case I5_ESPERAR_SINCRO:
      if (sensores.enfrente) {
        cout << "[INGENIERO] Cara a cara! Sincronizando INSTALL." << endl;
        paso_actual_i5++;
        A_paso = *paso_actual_i5;
        
        auto next_it = paso_actual_i5;
        next_it++;
        
        if (next_it != plan_tuberias.end()) {
            B_paso = *next_it;
            estado_i5 = I5_LLAMAR_TECNICO; 
        } else {
            cout << "[INGENIERO] ¡Tubería completada!" << endl;
            estado_i5 = I5_COMPLETADO;
        }
        return INSTALL;
      }
      return IDLE;
    
    case I5_COMPLETADO:
      return IDLE;
  }

  return IDLE;
}

/**
 * @brief Comportamiento del ingeniero para el Nivel 6.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
Action ComportamientoIngeniero::ComportamientoIngenieroNivel_6(Sensores sensores) {
  ActualizarMapa(sensores);
  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;

  int actual_conocido = 0;
  for(int i = 0; i < mapaResultado.size(); i++) {
      for(int j = 0; j < mapaResultado[0].size(); j++) {
          if(mapaResultado[i][j] != '?') actual_conocido++;
      }
  }

  if (exploracion_terminada_i6) {
      // 1. Guardamos una "foto" del estado ANTES de pensar
      EstadoIngenieroN5 estado_previo = estado_i5;
      
      Action accion_n5 = ComportamientoIngenieroNivel_5(sensores);

      // 2. CAMBIO CLAVE: Exigimos que ya estuviéramos en I5_IR_A o I5_IR_B en el turno anterior
      if (accion_n5 == IDLE && !hayPlan && 
         (estado_previo == I5_IR_A || estado_previo == I5_IR_B) && 
         (estado_i5 == I5_IR_A || estado_i5 == I5_IR_B)) {
          
          cout << "[INGENIERO N6] Camino bloqueado por niebla/obstaculo. Abortando..." << endl;
          exploracion_terminada_i6 = false; 
          estado_i5 = I5_CALCULANDO_PLAN;   
          plan_tuberias.clear();            
          mapa_conocido_tub_i6 = actual_conocido; 
          
          return ExploracionGuiadaNivel6(sensores); // <--- CAMBIO AQUÍ
      } 
      
      return accion_n5;
  }

  // --- FASE DE PLANIFICACIÓN DE TUBERÍAS ---
  bool listo_para_planear = false;
  if (sensores.BelPosF >= 0 && sensores.BelPosF < mapaResultado.size()) {
     for(int i = 0; i < mapaResultado.size(); i++) {
        for(int j = 0; j < mapaResultado[0].size(); j++) {
           if(mapaResultado[i][j] == 'U') { listo_para_planear = true; break; }
        }
        if(listo_para_planear) break;
     }
  }

  ciclos_exploracion_i6++;
  if (listo_para_planear && ciclos_exploracion_i6 % 20 == 0) {
      if (actual_conocido > mapa_conocido_tub_i6) {
          plan_tuberias = AEstrellaTuberias(sensores.BelPosF, sensores.BelPosC, 
                                            sensores.max_ecologico, mapaResultado, mapaCotas);
          if (!plan_tuberias.empty()) {
            VisualizaRedTuberias(plan_tuberias);
              exploracion_terminada_i6 = true;
              estado_i5 = I5_CALCULANDO_PLAN; 
              hayPlan = false; plan.clear(); 
              return IDLE; 
          } else {
              mapa_conocido_tub_i6 = actual_conocido;
          }
      }
  }

  return ExploracionGuiadaNivel6(sensores); // <--- CAMBIO AQUÍ
}

// =========================================================================
// FUNCIONES PROPORCIONADAS
// =========================================================================

/**
 * @brief Actualiza el mapaResultado y mapaCotas con la información de los sensores.
 * @param sensores Datos actuales de los sensores.
 */
void ComportamientoIngeniero::ActualizarMapa(Sensores sensores)
{
  mapaResultado[sensores.posF][sensores.posC] = sensores.superficie[0];
  mapaCotas[sensores.posF][sensores.posC] = sensores.cota[0];

  int pos = 1;
  switch (sensores.rumbo)
  {
  case norte:
    for (int j = 1; j < 4; j++)
      for (int i = -j; i <= j; i++)
      {
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
      for (int i = -j; i <= j; i++)
      {
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
      for (int i = -j; i <= j; i++)
      {
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
      for (int i = -j; i <= j; i++)
      {
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
 * @brief Determina si una casilla es transitable para el ingeniero.
 * @param f Fila de la casilla.
 * @param c Columna de la casilla.
 * @param tieneZapatillas Indica si el agente posee las zapatillas.
 * @return true si la casilla es transitable (no es muro ni precipicio).
 */
bool ComportamientoIngeniero::EsCasillaTransitableLevel0(int f, int c, bool tieneZapatillas)
{
  if (f < 0 || f >= mapaResultado.size() || c < 0 || c >= mapaResultado[0].size())
    return false;
  return es_camino(mapaResultado[f][c]); // Solo 'C', 'D', 'U' son transitables en Nivel 0
}

bool ComportamientoIngeniero::EsCasillaTransitable(int f, int c, bool tieneZapatillas) {
  if (f < 0 || f >= mapaResultado.size() || c < 0 || c >= mapaResultado[0].size())
    return false;

  return (mapaResultado[f][c] != 'P' && mapaResultado[f][c] != 'B' && mapaResultado[f][c] != 'M');
}

/**
 * @brief Comprueba si la casilla de delante es accesible por diferencia de altura.
 * Para el ingeniero: desnivel máximo 1 sin zapatillas, 2 con zapatillas.
 * @param actual Estado actual del agente (fila, columna, orientacion, zap).
 * @return true si el desnivel con la casilla de delante es admisible.
 */
bool ComportamientoIngeniero::EsAccesiblePorAltura(const ubicacion &actual, bool zap)
{
  ubicacion del = Delante(actual);
  if (del.f < 0 || del.f >= mapaCotas.size() || del.c < 0 || del.c >= mapaCotas[0].size())
    return false;
  int desnivel = abs(mapaCotas[del.f][del.c] - mapaCotas[actual.f][actual.c]);
  if (zap && desnivel > 2)
    return false;
  if (!zap && desnivel > 1)
    return false;
  return true;
}

/**
 * @brief Devuelve la posición (fila, columna) de la casilla que hay delante del agente.
 * Calcula la casilla frontal según la orientación actual (8 direcciones).
 * @param actual Estado actual del agente (fila, columna, orientacion).
 * @return Estado con la fila y columna de la casilla de enfrente.
 */
ubicacion ComportamientoIngeniero::Delante(const ubicacion &actual) const
{
  ubicacion delante = actual;
  switch (actual.brujula)
  {
  case 0:
    delante.f--;
    break; // norte
  case 1:
    delante.f--;
    delante.c++;
    break; // noreste
  case 2:
    delante.c++;
    break; // este
  case 3:
    delante.f++;
    delante.c++;
    break; // sureste
  case 4:
    delante.f++;
    break; // sur
  case 5:
    delante.f++;
    delante.c--;
    break; // suroeste
  case 6:
    delante.c--;
    break; // oeste
  case 7:
    delante.f--;
    delante.c--;
    break; // noroeste
  }
  return delante;
}

/**
 * @brief Imprime por consola la secuencia de acciones de un plan.
 *
 * @param plan  Lista de acciones del plan.
 */
void ComportamientoIngeniero::PintaPlan(const list<Action> &plan)
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
 * @brief Imprime las coordenadas y operaciones de un plan de tubería.
 *
 * @param plan  Lista de pasos (fila, columna, operación),
 *              donde operacion = -1 (DIG), operación = 1 (RAISE).
 */
void ComportamientoIngeniero::PintaPlan(const list<Paso> &plan)
{
  auto it = plan.begin();
  while (it != plan.end())
  {
    cout << it->fil << ", " << it->col << " (" << it->op << ")\n";
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
void ComportamientoIngeniero::VisualizaPlan(const ubicacion &st,
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

/**
 * @brief Convierte un plan de tubería en la lista de casillas usada
 *        por el sistema de visualización.
 *
 * @param st    Estado de partida (no utilizado directamente).
 * @param plan  Lista de pasos del plan de tubería.
 */
void ComportamientoIngeniero::VisualizaRedTuberias(const list<Paso> &plan)
{
  listaCanalizacionTuberias.clear();
  auto it = plan.begin();
  while (it != plan.end())
  {
    listaCanalizacionTuberias.push_back({it->fil, it->col, it->op});
    it++;
  }
}

/**
 * @brief Comprueba si el movimiento a la casilla de delante es viable, según sea esta transitable y la altura que tenga
 * @param actual    Casilla actual del agente
 * @param zap  Booleano que es true si tiene las zapatillas
 * @return True si la casilla es transitable y tiene un desnivel aceptable
 */
bool ComportamientoIngeniero::AndarViable0(const ubicacion &actual, bool zap) {
  bool viable;

  viable = EsCasillaTransitableLevel0(Delante(actual).f, Delante(actual).c, zap) && EsAccesiblePorAltura(actual, zap);

  return viable;
}

bool ComportamientoIngeniero::AndarViable(const ubicacion &actual, bool zap) {
  bool viable;

  viable = EsCasillaTransitable(Delante(actual).f, Delante(actual).c, zap) && EsAccesiblePorAltura(actual, zap);

  return viable;
}

/**
 * @brief Comprueba si el salto a la casilla es viable, según sean transitables la casilla y la intermedia y
 * la diferencia de alturas
 * @param actual Casilla actual del agente
 * @param zap True si tiene las zapatillas
 * @return True si el movimiento es viable
 */
bool ComportamientoIngeniero::SaltarViable0(const ubicacion &actual, bool zap) {
  bool viable = true;

  // Calculamos las dos casillas implicadas
  ubicacion intermedia = Delante(actual);
  ubicacion destino = Delante(intermedia);
  
  // 1. La casilla intermedia DEBE ser transitable
  if (!EsCasillaTransitable(intermedia.f, intermedia.c, zap)) viable = false;

  // 2. La casilla destino DEBE ser transitable
  if (!EsCasillaTransitableLevel0(destino.f, destino.c, zap)) viable =  false;

  // 3. Evaluamos la altura entre DONDE ESTAMOS (actual) y DONDE CAEMOS (destino)
  // El guión dice: "inferior a 2 sin zapatillas" (<= 1) e "inferior a 3 con zapatillas" (<= 2)
  int desnivel = abs(mapaCotas[destino.f][destino.c] - mapaCotas[actual.f][actual.c]);
  
  if (zap && desnivel > 2) viable =  false;
  if (!zap && desnivel > 1) viable = false;

  return viable;
}

bool ComportamientoIngeniero::SaltarViable(const ubicacion &actual, bool zap) {
  bool viable = true;

  // Calculamos las dos casillas implicadas
  ubicacion intermedia = Delante(actual);
  ubicacion destino = Delante(intermedia);
  
  // 1. La casilla intermedia DEBE ser transitable
  if (!EsCasillaTransitable(intermedia.f, intermedia.c, zap)) viable = false;

  // 2. La casilla destino DEBE ser transitable
  if (!EsCasillaTransitable(destino.f, destino.c, zap)) viable =  false;

  // 3. Evaluamos la altura entre DONDE ESTAMOS (actual) y DONDE CAEMOS (destino)
  // El guión dice: "inferior a 2 sin zapatillas" (<= 1) e "inferior a 3 con zapatillas" (<= 2)
  int desnivel = abs(mapaCotas[destino.f][destino.c] - mapaCotas[actual.f][actual.c]);
  
  if (zap && desnivel > 2) viable =  false;
  if (!zap && desnivel > 1) viable = false;
  
  return viable;
}

EstadoI ComportamientoIngeniero::applyI(Action accion, const EstadoI &st) {
    EstadoI next = st;

    switch (accion) {
        case WALK:
            if (AndarViable(st.site, st.zapatillas)) {
                next.site = Delante(st.site);

                // Bloquear el paso a casillas desconocidas en el planificador
                if (mapaResultado[next.site.f][next.site.c] == '?') return st;
                // Si caemos en unas zapatillas, las recogemos
                if (mapaResultado[next.site.f][next.site.c] == 'D') {
                    next.zapatillas = true;
                }
            }
            break;
            
        case JUMP:
            if (SaltarViable(st.site, st.zapatillas)) {
                next.site = Delante(Delante(st.site));

                // Bloquear el salto a casillas desconocidas
                if (mapaResultado[next.site.f][next.site.c] == '?') return st;
                // Si caemos en unas zapatillas tras el salto, las recogemos
                if (mapaResultado[next.site.f][next.site.c] == 'D') {
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

list<Action> ComportamientoIngeniero::B_AnchuraIngeniero(const EstadoI &inicio, const EstadoI &final) {
  NodoI current_node;
  list<NodoI> frontier;
  set<NodoI> explored;
  list<Action> path;

  current_node.estado = inicio;
  frontier.push_back(current_node);

  bool SolutionFound = (current_node.estado.site.f == final.site.f && current_node.estado.site.c == final.site.c);

  while (!SolutionFound && !frontier.empty()) {
    frontier.pop_front();
    explored.insert(current_node);

    // Acciones posibles para el Ingeniero
    Action acciones[] = {WALK, JUMP, TURN_SR, TURN_SL};
    
    for (Action accion : acciones) {
      NodoI child = current_node;
      child.estado = applyI(accion, current_node.estado);

      // Si el estado es el mismo, significa que el movimiento chocó con un muro/precipicio. Lo ignoramos.
      if (child.estado == current_node.estado) continue;

      // Comprobamos si hemos llegado a la meta
      if (child.estado.site.f == final.site.f && child.estado.site.c == final.site.c) {
        child.secuencia.push_back(accion);
        current_node = child;
        SolutionFound = true;
        break;
      }

      // Si no es la meta y no lo hemos explorado, lo añadimos a la frontera
      if (explored.find(child) == explored.end()) {
        child.secuencia.push_back(accion);
        frontier.push_back(child);
      }
    }

    // Seleccionamos el siguiente nodo a evaluar purgando los que ya estén en 'explored'
    if (!SolutionFound && !frontier.empty()) {
      current_node = frontier.front();
      while (explored.find(current_node) != explored.end() && !frontier.empty()) {
        frontier.pop_front();
        if (!frontier.empty()) current_node = frontier.front();
      }
    }
  }

  if (SolutionFound) {
      path = current_node.secuencia;
  }

  return path;
}

int ComportamientoIngeniero::CalcularImpactoEco(unsigned char terreno, int op) {
  int coste = 0;
  
  // 1. Coste de la acción INSTALL (Poner la tubería)
  if (terreno == 'A') coste += 50;
  else if (terreno == 'H') coste += 45;
  else if (terreno == 'S') coste += 25;
  else if (terreno == 'C' || terreno == 'U') coste += 15;
  else coste += 30; // Resto

  // 2. Coste de modificar el terreno (RAISE o DIG)
  if (op == 1) { // RAISE
    if (terreno == 'H') coste += 55;
    else if (terreno == 'S') coste += 30;
    else if (terreno == 'C' || terreno == 'U') coste += 10;
    else coste += 40;
  } else if (op == -1) { // DIG
    if (terreno == 'H') coste += 65;
    else if (terreno == 'S') coste += 40;
    else if (terreno == 'C' || terreno == 'U') coste += 25;
    else coste += 50;
  }

  return coste;
}

list<Paso> ComportamientoIngeniero::AEstrellaTuberias(int inicio_f, int inicio_c, int limite_eco,
                                                      const vector<vector<unsigned char>> &terreno,
                                                      const vector<vector<unsigned char>> &altura) {
  vector<pair<int, int>> metas;
  for (int i = 0; i < terreno.size(); i++) {
    for (int j = 0; j < terreno[0].size(); j++) {
      if (terreno[i][j] == 'U') metas.push_back({i, j});
    }
  }

  auto calcular_h = [&](int f, int c) {
    int min_dist = 999999;
    for (auto m : metas) {
      int dist = abs(f - m.first) + abs(c - m.second);
      if (dist < min_dist) min_dist = dist;
    }
    return min_dist;
  };

  priority_queue<NodoTubo, vector<NodoTubo>, ComparaNodosTubo> frontier;
  map<tuple<int, int, int>, int> min_eco_expanded; 
  list<Paso> solucion;

  unsigned char terr_inicio = terreno[inicio_f][inicio_c];
  int alt_inicio = altura[inicio_f][inicio_c];
  
  int ops_iniciales[] = {0, 1, -1};
  for (int op : ops_iniciales) {
    if ((op != 0) && (terr_inicio == 'A' || terr_inicio == 'M' || terr_inicio == 'P')) continue;
    if (op == 1 && alt_inicio >= 9) continue;
    if (op == -1 && alt_inicio <= 1) continue;

    int coste_mod = CalcularImpactoEco(terr_inicio, op) - CalcularImpactoEco(terr_inicio, 0);
    
    // Barrera de peaje
    if (coste_mod <= limite_eco) {
      NodoTubo start_node;
      start_node.estado = {inicio_f, inicio_c, alt_inicio + op, coste_mod};
      start_node.plan.push_back({inicio_f, inicio_c, op});
      start_node.tramos = 1;
      start_node.f_cost = 1 + calcular_h(inicio_f, inicio_c);
      frontier.push(start_node);
    }
  }

  int df[] = {-1, 1, 0, 0};
  int dc[] = {0, 0, -1, 1};

  int nodos_expandidos = 0;

  while (!frontier.empty()) {
    nodos_expandidos++;

    if (nodos_expandidos > 10000)
      return list<Paso>();
    
    
    NodoTubo actual = frontier.top();
    frontier.pop();

    tuple<int, int, int> estado_clave = {actual.estado.f, actual.estado.c, actual.estado.altura_mod};

    // Poda pareto
    if (min_eco_expanded.find(estado_clave) != min_eco_expanded.end()) {
        if (actual.estado.eco_acumulado >= min_eco_expanded[estado_clave]) {
            continue; 
        }
    }
    min_eco_expanded[estado_clave] = actual.estado.eco_acumulado;

    if (terreno[actual.estado.f][actual.estado.c] == 'U') {
        solucion = actual.plan;
        break; 
    }

    for (int i = 0; i < 4; i++) {
      int nf = actual.estado.f + df[i];
      int nc = actual.estado.c + dc[i];

      if (nf < 0 || nf >= terreno.size() || nc < 0 || nc >= terreno[0].size()) continue;

      unsigned char n_terr = terreno[nf][nc];
      if (n_terr == 'M' || n_terr == 'P' || n_terr == 'B' || n_terr == '?') continue;

      int n_alt_orig = altura[nf][nc];
      
      for (int op : ops_iniciales) {
        if ((op != 0) && (n_terr == 'A')) continue;
        if (op == 1 && n_alt_orig >= 9) continue;
        if (op == -1 && n_alt_orig <= 1) continue;

        int n_alt_mod = n_alt_orig + op;
        
        // Gravedad
        if (n_alt_mod == actual.estado.altura_mod || n_alt_mod == actual.estado.altura_mod - 1) {
            
          int coste_install_origen = CalcularImpactoEco(terreno[actual.estado.f][actual.estado.c], 0);
          int coste_destino = CalcularImpactoEco(n_terr, op); // Tu función ya devuelve INSTALL + MOD
          
          int coste_segmento = coste_install_origen + coste_destino;
          int nuevo_eco = actual.estado.eco_acumulado + coste_segmento;

          if (nuevo_eco <= limite_eco) {
            NodoTubo hijo;
            hijo.estado = {nf, nc, n_alt_mod, nuevo_eco};
            hijo.plan = actual.plan;
            hijo.plan.push_back({nf, nc, op});
            hijo.tramos = actual.tramos + 1;
            hijo.f_cost = hijo.tramos + calcular_h(nf, nc);

            tuple<int, int, int> clave_hijo = {nf, nc, n_alt_mod};
            if (min_eco_expanded.find(clave_hijo) == min_eco_expanded.end() || 
              nuevo_eco < min_eco_expanded[clave_hijo]) {
              frontier.push(hijo);
            }
          }
        }
      }
    }
  }

  return solucion;
}

/**
 * @brief Exploración específica para el Nivel 6. 
 * Fase 1: Misil fluido. Va a la Belkanita pero fluye ágilmente por los obstáculos y evita el agua a toda costa.
 * Fase 2: Modo Gacela (Líneas rectas largas para descubrir la 'U').
 */
Action ComportamientoIngeniero::ExploracionGuiadaNivel6(Sensores sensores)
{
  Action accion = IDLE;

  ActualizarMapa(sensores);

  if (sensores.superficie[0] == 'D') tiene_zapatillas = true;

  mapa_visitas[{sensores.posF, sensores.posC}]++;

  int prioridades[6];
  ubicacion ubicaciones[6];
  ubicacion actual = {sensores.posF, sensores.posC, sensores.rumbo};
  Orientacion izq = static_cast<Orientacion>((sensores.rumbo + 7) % 8);
  Orientacion der = static_cast<Orientacion>((sensores.rumbo + 1) % 8);
  ubicacion mira_izq = {sensores.posF, sensores.posC, izq};
  ubicacion mira_der = {sensores.posF, sensores.posC, der};

  ubicaciones[0] = mira_izq;
  ubicaciones[1] = actual;
  ubicaciones[2] = mira_der;
  ubicaciones[3] = mira_izq;
  ubicaciones[4] = actual;
  ubicaciones[5] = mira_der;

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
      int base_prioridad = visitas * 10000; // Las visitas dominan la decisión
      
      if (sup == 'D' && !tiene_zapatillas) {
        prioridades[i] = base_prioridad - 60000; // Queremos las zapatillas
      } 
      else if (sup == 'C' || sup == 'U') {
        prioridades[i] = base_prioridad - 50000;   // Preferimos camino a hierba
      }
      else if (sup == 'S') {
        prioridades[i] = base_prioridad - 30000;
      }
      else if (sup == 'A') {
        prioridades[i] = base_prioridad + 20000; // Penalizamos mucho el agua
      } 
      else {
        prioridades[i] = base_prioridad;        // Hierba, o Zapatillas si ya las tenemos
      }

      // 1. Si está construyendo, va hacia la tubería
      bool visitada_belkanita = (sensores.BelPosF != -1 && mapa_visitas[{sensores.BelPosF, sensores.BelPosC}] > 0);

      // 1. Si está construyendo, va hacia la tubería
      if ((estado_i5 == I5_IR_A || estado_i5 == I5_IR_B) && exploracion_terminada_i6) {
        int target_f = (estado_i5 == I5_IR_A) ? A_paso.fil : B_paso.fil;
        int target_c = (estado_i5 == I5_IR_A) ? A_paso.col : B_paso.col;
        
        if (abs(destino.f - target_f) + abs(destino.c - target_c) < abs(sensores.posF - target_f) + abs(sensores.posC - target_c)) {
          prioridades[i] -= 40000; // Prioridad aumentada
        }
      } 
      // 2. Si NO ha visitado la belkanita, va directo a ella
      else if (!visitada_belkanita && sensores.BelPosF != -1) {
        if (abs(destino.f - sensores.BelPosF) + abs(destino.c - sensores.BelPosC) < abs(sensores.posF - sensores.BelPosF) + abs(sensores.posC - sensores.BelPosC)) {
          prioridades[i] -= 40000; // Misil activado (supera penalización de agua/arena)
        }
      }
    }
  }

  // --- BUCLE DE SALTAR ---
  for (int i = 3; i < 6; i++) {
    ubicacion destino = Delante(Delante(ubicaciones[i]));
    unsigned char sup = sensores.superficie[2*i-2];

    if (sensores.agentes[2*i-2] != '_') {
      mapa_visitas[{destino.f, destino.c}] += 5; 
    }

    int visitas = mapa_visitas[{destino.f, destino.c}];

    if (!SaltarViable(ubicaciones[i], tiene_zapatillas) || sensores.agentes[2*i-2] != '_' || sensores.agentes[i-2] != '_') {
      prioridades[i] = 999999;
    }
    else {
      int base_prioridad = visitas * 10000;
      
      if (sup == 'D' && !tiene_zapatillas) {
        prioridades[i] = base_prioridad - 65000;
      } 
      else if (sup == 'C' || sup == 'U') {
        prioridades[i] = base_prioridad - 55000;
      }
      else if (sup == 'S') {
        prioridades[i] = base_prioridad - 35000;
      }
      else if (sup == 'A') {
        prioridades[i] = base_prioridad + 15000;
      } 
      else {
        prioridades[i] = base_prioridad;
      }

      bool visitada_belkanita = (sensores.BelPosF != -1 && mapa_visitas[{sensores.BelPosF, sensores.BelPosC}] > 0);

      // 1. Si está construyendo, va hacia la tubería
      if ((estado_i5 == I5_IR_A || estado_i5 == I5_IR_B) && exploracion_terminada_i6) {
        int target_f = (estado_i5 == I5_IR_A) ? A_paso.fil : B_paso.fil;
        int target_c = (estado_i5 == I5_IR_A) ? A_paso.col : B_paso.col;
        
        if (abs(destino.f - target_f) + abs(destino.c - target_c) < abs(sensores.posF - target_f) + abs(sensores.posC - target_c)) {
          prioridades[i] -= 45000; // Prioridad aumentada
        }
      } 
      // 2. Si NO ha visitado la belkanita, va directo a ella
      else if (!visitada_belkanita && sensores.BelPosF != -1) {
        if (abs(destino.f - sensores.BelPosF) + abs(destino.c - sensores.BelPosC) < abs(sensores.posF - sensores.BelPosF) + abs(sensores.posC - sensores.BelPosC)) {
          prioridades[i] -= 45000; // Misil activado (supera penalización de agua/arena)
        }
      }
    }
  }

  int inercia = (sensores.nivel == 6) ? 6000 : 1;

  if (prioridades[1] < 999999) prioridades[1] -= inercia; // WALK
  if (prioridades[4] < 999999) prioridades[4] -= inercia; // JUMP

  prioridades[0] -= 2; 
  prioridades[3] -= 2;

  int min_frente = *min_element(prioridades, prioridades + 6);

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
        
        if (sup == 'D' && !tiene_zapatillas) prio_lateral -= 60000;
        else if (sup == 'C' || sup == 'U') prio_lateral -= 50000;
        else if (sup == 'S') prio_lateral -= 30000;
        else if (sup == 'A') prio_lateral += 20000;

        // --- NUEVO: Fuerza de gravedad del GPS para el radar ---
        bool visitada_belkanita = (sensores.BelPosF != -1 && mapa_visitas[{sensores.BelPosF, sensores.BelPosC}] > 0);
        
        if ((estado_i5 == I5_IR_A || estado_i5 == I5_IR_B) && exploracion_terminada_i6) {
           int target_f = (estado_i5 == I5_IR_A) ? A_paso.fil : B_paso.fil;
           int target_c = (estado_i5 == I5_IR_A) ? A_paso.col : B_paso.col;
           if (abs(dest_radar.f - target_f) + abs(dest_radar.c - target_c) < abs(sensores.posF - target_f) + abs(sensores.posC - target_c)) {
             prio_lateral -= 30000;
           }
        } 
        else if (!visitada_belkanita && sensores.BelPosF != -1) {
           // Si el movimiento lateral nos ACERCA a la Belkanita, que tire con fuerza
           if (abs(dest_radar.f - sensores.BelPosF) + abs(dest_radar.c - sensores.BelPosC) < abs(sensores.posF - sensores.BelPosF) + abs(sensores.posC - sensores.BelPosC)) {
             prio_lateral -= 55000; 
           }
        }
        // --------------------------------------------------------

        // Si el lateral es ESTRICTAMENTE MEJOR que lo que tenemos delante
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

  int min = distance(prioridades, min_element(prioridades, prioridades + 6));

  // Si acabamos de girar a un lado, prohibimos girar al lado contrario inmediatamente
  // para romper cualquier bucle de Izquierda-Derecha.
  if (min == 0 && last_action == TURN_SR) {
    prioridades[0] = 999999;
    min = distance(prioridades, min_element(prioridades, prioridades + 6));
  } else if (min == 2 && last_action == TURN_SL) {
    prioridades[2] = 999999;
    min = distance(prioridades, min_element(prioridades, prioridades + 6));
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
  case 3:
    accion = TURN_SL;
    break;
  case 4:
    accion = JUMP;
    break;
  case 5:
    accion = TURN_SR;
    break;
  }

  last_action = accion;
  return accion;
}