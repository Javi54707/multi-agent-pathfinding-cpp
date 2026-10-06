#ifndef COMPORTAMIENTOINGENIERO_H
#define COMPORTAMIENTOINGENIERO_H

#include <chrono>
#include <list>
#include <map>
#include <set>
#include <thread>
#include <time.h>
#include <utility>

#include "comportamientos/comportamiento.hpp"

struct EstadoI {
  ubicacion site;
  bool zapatillas;

  // Sobrecarga del operador == para comparar estados
  bool operator==(const EstadoI &st) const {
    return site.f == st.site.f && 
            site.c == st.site.c && 
            site.brujula == st.site.brujula && 
            zapatillas == st.zapatillas;
  }
};

struct NodoI {
  EstadoI estado;
  std::list<Action> secuencia; // Camino de acciones hasta este nodo

  // Sobrecarga para usar Find o set
  bool operator==(const NodoI &node) const {
      return estado == node.estado;
  }

  // Sobrecarga del operador < para poder usar std::set en la lista de 'explored'
  // Es vital para que la búsqueda sea O(log N) y no tarde una eternidad.
  bool operator<(const NodoI &node) const {
    if (estado.site.f < node.estado.site.f) return true;
    else if (estado.site.f == node.estado.site.f && estado.site.c < node.estado.site.c) return true;
    else if (estado.site.f == node.estado.site.f && estado.site.c == node.estado.site.c && estado.site.brujula < node.estado.site.brujula) return true;
    else if (estado.site.f == node.estado.site.f && estado.site.c == node.estado.site.c && estado.site.brujula == node.estado.site.brujula && estado.zapatillas < node.estado.zapatillas) return true;
    else return false;
  }
};

struct EstadoTubo {
  int f;
  int c;
  int altura_mod;
  int eco_acumulado;
};

struct NodoTubo {
  EstadoTubo estado;
  std::list<Paso> plan;
  int tramos;
  int f_cost;
};

// Comparador para que la priority_queue actúe como un Min-Heap
struct ComparaNodosTubo {
  bool operator()(const NodoTubo& a, const NodoTubo& b) const {
    // Principalmente ordenamos por el menor f_cost
    if (a.f_cost != b.f_cost) return a.f_cost > b.f_cost; 
    // En caso de empate, priorizamos el camino que haya generado menor impacto ecológico
    return a.estado.eco_acumulado > b.estado.eco_acumulado; 
  }
};

class ComportamientoIngeniero : public Comportamiento {
public:
  // =========================================================================
  // CONSTRUCTORES
  // =========================================================================
  
  /**
   * @brief Constructor para niveles 0, 1 y 6 (sin mapa completo)
   * @param size Tamaño del mapa (si es 0, se inicializa más tarde)
   */
  ComportamientoIngeniero(unsigned int size = 0) : Comportamiento(size) {
    // Inicializar Variables de Estado
    last_action = IDLE;
    tiene_zapatillas = false;
  }

  /**
   * @brief Constructor para niveles 2, 3, 4 y 5 (con mapa completo conocido)
   * @param mapaR Mapa de terreno conocido
   * @param mapaC Mapa de cotas conocido
   */
  ComportamientoIngeniero(std::vector<std::vector<unsigned char>> mapaR, 
                         std::vector<std::vector<unsigned char>> mapaC): 
                         Comportamiento(mapaR, mapaC) {
    last_action = IDLE;
    tiene_zapatillas = false;
  }

  ComportamientoIngeniero(const ComportamientoIngeniero &comport): Comportamiento(comport) {}
  ~ComportamientoIngeniero() {}

  /**
   * @brief Bucle principal de decisión del agente.
   * Estudia los sensores y decide la siguiente acción.
   * 
   * EJEMPLO DE USO:
   * Action accion = think(sensores);
   * return accion; // El motor ejecutará esta acción
   */
  Action think(Sensores sensores);

  ComportamientoIngeniero *clone() {
    return new ComportamientoIngeniero(*this);
  }

  // =========================================================================
  // ÁREA DE IMPLEMENTACIÓN DEL ESTUDIANTE
  // =========================================================================

  // Funciones específicas para cada nivel (para ser implementadas por el alumno)
  
  /**
   * @brief Implementación del Nivel 0.
   * @param sensores Datos actuales de los sensores del agente.
   * @return Acción a realizar.
   */
  Action ComportamientoIngenieroNivel_0(Sensores sensores);
  
  /**
   * @brief Implementación del Nivel 1.
   * @param sensores Datos actuales de los sensores del agente.
   * @return Acción a realizar.
   */
  Action ComportamientoIngenieroNivel_1(Sensores sensores);
  
  /**
   * @brief Implementación del Nivel 2.
   * @param sensores Datos actuales de los sensores del agente.
   * @return Acción a realizar.
   */ 
  Action ComportamientoIngenieroNivel_2(Sensores sensores);
  
  /**
   * @brief Implementación del Nivel 3.
   * @param sensores Datos actuales de los sensores del agente.
   * @return Acción a realizar.
   */
  Action ComportamientoIngenieroNivel_3(Sensores sensores);
  
  /**
   * @brief Implementación del Nivel 4.
   * @param sensores Datos actuales de los sensores del agente.
   * @return Acción a realizar.
   */
  Action ComportamientoIngenieroNivel_4(Sensores sensores);
  
  /**
   * @brief Implementación del Nivel 5.
   * @param sensores Datos actuales de los sensores del agente.
   * @return Acción a realizar.
   */
  Action ComportamientoIngenieroNivel_5(Sensores sensores);
  
  /**
   * @brief Implementación del Nivel 6.
   * @param sensores Datos actuales de los sensores del agente.
   * @return Acción a realizar.
   */
  Action ComportamientoIngenieroNivel_6(Sensores sensores);

protected:
  // =========================================================================
  // FUNCIONES PROPORCIONADAS
  // =========================================================================

  /**
   * @brief Actualiza la información del mapa interno basándose en los sensores.
   * IMPORTANTE: Esta función ya está implementada. Actualiza mapaResultado y mapaCotas
   * con la información de los 16 sensores (casilla actual + 15 casillas alrededor).
   */
  void ActualizarMapa(Sensores sensores);

  /**
   * @brief Comprueba si una casilla es transitable.
   * @param f Fila de la casilla.
   * @param c Columna de la casilla.
   * @param tieneZapatillas Indica si el agente posee zapatillas.
   * @return true si la casilla es transitable (no es muro ni precipicio).
   */
  bool EsCasillaTransitableLevel0(int f, int c, bool tieneZapatillas);

  /**
   * @brief Comprueba si la casilla de delante es accesible por diferencia de altura.
   * REGLAS: Desnivel máximo 1 sin zapatillas, 2 con zapatillas.
   * @param actual Estado actual del agente (fila, columna, orientacion).
   * @return true si el desnivel con la casilla de delante es admisible.
   */
  bool EsAccesiblePorAltura(const ubicacion &actual, bool zap);

  /**
   * @brief Devuelve la posición (fila, columna) de la casilla que hay delante del agente.
   * @param actual Estado actual del agente (fila, columna, orientacion).
   * @return Estado con la fila y columna de la casilla de enfrente.
   */
  ubicacion Delante(const ubicacion &actual) const;

  bool es_camino(unsigned char c) const;

  /**
 * @brief Imprime por consola la secuencia de acciones de un plan para un agente.
 * @param plan  Lista de acciones del plan.
 */
  void PintaPlan(const list<Action> &plan);


/**
 * @brief Imprime las coordenadas y operaciones de un plan de tubería.
 * @param plan  Lista de pasos (fila, columna, operación).
 */
  void PintaPlan(const list<Paso> &plan);


  /**
 * @brief Convierte un plan de acciones en una lista de casillas para
 *        su visualización en el mapa gráfico.
 * @param st    Estado de partida.
 * @param plan  Lista de acciones del plan.
 */
  void VisualizaPlan(const ubicacion &st, const list<Action> &plan);

  /**
 * @brief Convierte un plan de tubería en la lista de casillas usada
 *        por el sistema de visualización.
 * @param st    Estado de partida (no utilizado directamente).
 * @param plan  Lista de pasos del plan de tubería.
 */
  void VisualizaRedTuberias(const list<Paso> &plan);



private:
  // =========================================================================
  // VARIABLES DE ESTADO (PUEDEN SER EXTENDIDAS POR EL ALUMNO)
  // =========================================================================
  Action last_action;
  bool tiene_zapatillas;

  // Mapa para recordar cuántas veces hemos pasado por cada coordenada
  std::map<std::pair<int, int>, int> mapa_visitas;

  // Funciones auxiliares
  /**
 * @brief Comprueba si el movimiento a la casilla de delante es viable, según sea esta transitable y la altura que tenga
 * @param actual    Casilla actual del agente
 * @param zap  Booleano que es true si tiene las zapatillas
 * @return True si la casilla es transitable y tiene un desnivel aceptable
 */
  bool AndarViable0(const ubicacion &actual, bool zap);

  bool AndarViable(const ubicacion &actual, bool zap);

  /**
   * @brief Comprueba si el salto a la casilla es viable, según sean transitables la casilla y la intermedia y
   * la diferencia de alturas
   * @param actual Casilla actual del agente
   * @param zap True si tiene las zapatillas
   * @return True si el movimiento es viable
   */
  bool SaltarViable0(const ubicacion &actual, bool zap);

  bool SaltarViable(const ubicacion &actual, bool zap);

  bool EsCasillaTransitable(int f, int c, bool tieneZapatillas);

  // Variables para la ejecución de planes (Niveles deliberativos)
  bool hayPlan = false;
  std::list<Action> plan;

  // Funciones del Algoritmo de Búsqueda
  std::list<Action> B_AnchuraIngeniero(const EstadoI &inicio, const EstadoI &final);
                                       
  EstadoI applyI(Action accion, const EstadoI &st);

  //Variable Nivel 4
  std::list<Paso> plan_tuberias;
  int CalcularImpactoEco(unsigned char terreno, int op);
  std::list<Paso> AEstrellaTuberias(int inicio_f, int inicio_c, int limite_eco,
                                    const std::vector<std::vector<unsigned char>> &terreno,
                                    const std::vector<std::vector<unsigned char>> &altura);

  // --- NUEVAS VARIABLES NIVEL 5 ---
  enum EstadoIngenieroN5 {
    I5_CALCULANDO_PLAN,
    I5_IR_A,
    I5_ACONDICIONAR_A,
    I5_LLAMAR_TECNICO,
    I5_IR_B,
    I5_ACONDICIONAR_B,
    I5_MIRAR_A,
    I5_ESPERAR_SINCRO,
    I5_COMPLETADO
  };
  EstadoIngenieroN5 estado_i5 = I5_CALCULANDO_PLAN;
  std::list<Paso>::iterator paso_actual_i5;
  Paso A_paso;
  Paso B_paso;

  // --- VARIABLES NIVEL 6 ---
  bool exploracion_terminada_i6 = false;
  int ciclos_exploracion_i6 = 0;
  Action ExploracionGuiadaNivel6(Sensores sensores);
  
  // Puertas lógicas anti-spam de CPU
  int mapa_conocido_i6 = 0;
  int mapa_conocido_tub_i6 = 0;
};

#endif
