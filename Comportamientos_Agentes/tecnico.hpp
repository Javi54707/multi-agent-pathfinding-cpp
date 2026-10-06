#ifndef COMPORTAMIENTOTECNICO_H
#define COMPORTAMIENTOTECNICO_H

#include <chrono>
#include <time.h>
#include <thread>
#include <list>
#include <map>
#include <utility>
#include <set>

#include "comportamientos/comportamiento.hpp"

struct EstadoT {
  ubicacion site;
  bool zapatillas;

  bool operator==(const EstadoT &st) const {
    return site.f == st.site.f && 
            site.c == st.site.c && 
            site.brujula == st.site.brujula && 
            zapatillas == st.zapatillas;
  }

  bool operator<(const EstadoT &st) const {
    if (site.f < st.site.f) return true;
    else if (site.f == st.site.f && site.c < st.site.c) return true;
    else if (site.f == st.site.f && site.c == st.site.c && site.brujula < st.site.brujula) return true;
    else if (site.f == st.site.f && site.c == st.site.c && site.brujula == st.site.brujula && (!zapatillas && st.zapatillas)) return true;
    else return false;
  }
};

struct NodoT {
  EstadoT estado;
  std::list<Action> secuencia;
  int coste_g;      // Energía consumida desde el inicio
  int heuristica_h; // Estimación de energía hasta la meta

  // F(n) = G(n) + H(n)
  int f() const {
    return coste_g + heuristica_h;
  }

  bool operator==(const NodoT &node) const {
    return estado == node.estado;
  }

  // Operador < para el SET de Explorados (búsqueda rápida)
  bool operator<(const NodoT &node) const {
    if (estado.site.f < node.estado.site.f) return true;
    else if (estado.site.f == node.estado.site.f && estado.site.c < node.estado.site.c) return true;
    else if (estado.site.f == node.estado.site.f && estado.site.c == node.estado.site.c && estado.site.brujula < node.estado.site.brujula) return true;
    else if (estado.site.f == node.estado.site.f && estado.site.c == node.estado.site.c && estado.site.brujula == node.estado.site.brujula && (!estado.zapatillas && node.estado.zapatillas)) return true;
    else return false;
  }
};

// Comparador para la COLA DE PRIORIDAD (Min-Heap) del A*
// En C++, priority_queue saca el "mayor" por defecto, así que invertimos el operador
// para que el nodo con MENOR F(n) se expanda primero.
struct ComparaNodosT {
  bool operator()(const NodoT& a, const NodoT& b) const {
    if (a.f() != b.f()) return a.f() > b.f();
    if (a.coste_g != b.coste_g) return a.coste_g > b.coste_g; 
    
    return a.secuencia.size() > b.secuencia.size(); 
  }
};

// =========================================================================
// DOCUMENTACIÓN PARA ESTUDIANTES
// =========================================================================
/*
 * CLASE: ComportamientoTecnico
 * 
 * DESCRIPCIÓN:
 * Esta clase implementa el comportamiento del agente Técnico en el mundo Belkan.
 * El técnico colabora con el ingeniero para resolver el problema de instalación de tuberías
 */



class ComportamientoTecnico : public Comportamiento {
public:
  // =========================================================================
  // CONSTRUCTORES
  // =========================================================================
  
  /**
   * @brief Constructor para niveles 0, 1 y 6 (sin mapa completo)
   * @param size Tamaño del mapa (si es 0, se inicializa más tarde)
   */
  ComportamientoTecnico(unsigned int size = 0) : Comportamiento(size) {
    // Inicializar Variables de Estado
    last_action = IDLE;
    tiene_zapatillas = false;
  }

  /**
   * @brief Constructor para niveles 2, 3, 4 y 5 (con mapa completo conocido)
   * @param mapaR Mapa de terreno conocido
   * @param mapaC Mapa de cotas conocido
   */
  ComportamientoTecnico(std::vector<std::vector<unsigned char>> mapaR, 
                       std::vector<std::vector<unsigned char>> mapaC): 
                       Comportamiento(mapaR, mapaC) {
    last_action = IDLE;
    tiene_zapatillas = false;
  }

  ComportamientoTecnico(const ComportamientoTecnico &comport): Comportamiento(comport) {}
  ~ComportamientoTecnico() {}

  /**
   * @brief Bucle principal de decisión del técnico.
   * Estudia los sensores y decide la siguiente acción.
   * 
   * EJEMPLO DE USO:
   * Action accion = think(sensores);
   * return accion; // El motor ejecutará esta acción
   */
  Action think(Sensores sensores);

  ComportamientoTecnico *clone() {
    return new ComportamientoTecnico(*this);
  }

  // =========================================================================
  // ÁREA DE IMPLEMENTACIÓN DEL ESTUDIANTE
  // =========================================================================
  
/**
 * @brief Comportamiento del técnico para el Nivel 0.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
  Action ComportamientoTecnicoNivel_0(Sensores sensores);
  
/**
 * @brief Comportamiento del técnico para el Nivel 1.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
  Action ComportamientoTecnicoNivel_1(Sensores sensores);
  
/**
 * @brief Comportamiento del técnico para el Nivel 2.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
  Action ComportamientoTecnicoNivel_2(Sensores sensores);
  
/**
 * @brief Comportamiento del técnico para el Nivel 3.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
  Action ComportamientoTecnicoNivel_3(Sensores sensores);
  
/**
 * @brief Comportamiento del técnico para el Nivel 4.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
  Action ComportamientoTecnicoNivel_4(Sensores sensores);
  
/**
 * @brief Comportamiento del técnico para el Nivel 5.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
  Action ComportamientoTecnicoNivel_5(Sensores sensores);
  
/**
 * @brief Comportamiento del técnico para el Nivel 6.
 * @param sensores Datos actuales de los sensores.
 * @return Acción a realizar.
 */
  Action ComportamientoTecnicoNivel_6(Sensores sensores);

protected:
  // =========================================================================
  // FUNCIONES PROPORCIONADAS
  // =========================================================================

  /**
   * @brief Actualiza el mapaResultado y mapaCotas con la información de los sensores.
   * IMPORTANTE: Esta función ya está implementada. Actualiza mapaResultado y mapaCotas
   * con la información de los 16 sensores.
   */
  void ActualizarMapa(Sensores sensores);

  /**
   * @brief Determina si una casilla es transitable para el técnico.
   * NOTA: El técnico puede tener reglas de transitabilidad diferentes al ingeniero.
   * @param f Fila de la casilla.
   * @param c Columna de la casilla.
   * @param tieneZapatillas Indica si el agente posee las zapatillas.
   * @return true si la casilla es transitable.
   */
  bool EsCasillaTransitableLevel0(int f, int c, bool tieneZapatillas);

  bool EsCasillaTransitable(int f, int c, bool tieneZapatillas);

  /**
   * @brief Comprueba si la casilla de delante es accesible por diferencia de altura.
   * REGLA PARA TÉCNICO: Desnivel máximo siempre 1 (independiente de zapatillas).
   * @param actual Estado actual del agente (fila, columna, orientacion).
   * @return true si el desnivel con la casilla de delante es admisible.
   */
  bool EsAccesiblePorAltura(const ubicacion &actual);

  /**
   * @brief Devuelve la posición (fila, columna) de la casilla que hay delante del agente.
   * @param actual Estado actual del agente (fila, columna, orientacion).
   * @return Estado con la fila y columna de la casilla de enfrente.
   */
  ubicacion Delante(const ubicacion &actual) const;

  /**
   * @brief Comprueba si una celda es de tipo transitable por defecto.
   * @param c Carácter que representa el tipo de superficie.
   * @return true si es camino ('C'), zapatillas ('D') o meta ('U').
   */
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

private:
  // =========================================================================
  // VARIABLES DE ESTADO (PUEDEN SER EXTENDIDAS POR EL ALUMNO)
  // =========================================================================
  Action last_action;
  bool tiene_zapatillas;

  // Mapa para recordar las visitas a cada casilla
  std::map<std::pair<int, int>, int> mapa_visitas;

  // Funciones auxiliares del Técnico
  /**
  * @brief Comprueba si el movimiento a la casilla de delante es viable, según sea esta transitable y la altura que tenga
  * @param actual    Casilla actual del agente
  * @param zap  Booleano que es true si tiene las zapatillas
  * @return True si la casilla es transitable y tiene un desnivel aceptable
  */
  bool AndarViable0(const ubicacion &actual, bool zap);

  bool AndarViable(const ubicacion &actual, bool zap);

  // Variables para la ejecución de planes
  bool hayPlan = false;
  std::list<Action> plan;

  // Funciones del Algoritmo A*
  std::list<Action> AEstrellaTecnico(const EstadoT &inicio, const EstadoT &final, 
                                     const std::vector<std::vector<unsigned char>> &terreno, 
                                     const std::vector<std::vector<unsigned char>> &altura,
                                     const std::set<std::pair<int, int>> &obstaculos = std::set<std::pair<int, int>>(),
                                     int limite_nodos = -1);
                                       
  EstadoT applyT(Action accion, const EstadoT &st, 
                 const std::vector<std::vector<unsigned char>> &terreno, 
                 const std::vector<std::vector<unsigned char>> &altura,
                 const std::set<std::pair<int, int>> &obstaculos = std::set<std::pair<int, int>>());

  // Función para calcular el coste de energía de una acción
  int costeEnergiaT(Action accion, const EstadoT &st, 
                    const std::vector<std::vector<unsigned char>> &terreno, 
                    const std::vector<std::vector<unsigned char>> &altura);


  // --- VARIABLES NIVEL 5 ---
  enum EstadoTecnicoN5 {
    T5_ESPERANDO,
    T5_IR_DESTINO,
    T5_MIRAR_INGENIERO
  };
  EstadoTecnicoN5 estado_t5 = T5_ESPERANDO;

  // Guardamos las coordenadas para que no se nos borren
  int dest_f_t5 = -1;
  int dest_c_t5 = -1;

  // Memoria a corto plazo de obstáculos dinámicos (el ingeniero)
  std::set<std::pair<int, int>> obstaculos_t5;

  // --- VARIABLES NIVEL 6 ---
  bool construyendo_t6 = false;
  bool n5_stuck_t6 = false;
  int mapa_conocido_t6 = 0;
  Action ExploracionGuiadaNivel6(Sensores sensores);
};

#endif
