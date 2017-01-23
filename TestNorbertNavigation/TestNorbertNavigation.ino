/////////////////////////////////////////////////////////////
//
// Test NorbertNavigation
//
// Tests the various inputs of Norbert's base. 
//
// David Beaudette (2017)
//
/////////////////////////////////////////////////////////////

#include <Timer.h>
#include <Event.h>

/////////////////////////////////////////////////////////////
//
// Constantes
//
/////////////////////////////////////////////////////////////

#define DEBOGGAGE 1

// Encodeur de roue droite
const int encodeur_a = 2;
const int encodeur_b = 3;
const int encodeur_int_a = 0;
const int encodeur_int_b = 1;

// Numero de pin de l'entree de direction
const int mesure_direction = A6;

// Batterie
const int batterie = A4;

// Convertisseur analogique a numerique
const float adc_vref = 3.4f;
const float adc_max_f32 = 1023.0f;
const float adc_to_volts = adc_vref / adc_max_f32;

// Capteurs de distance
const int ir_avant   = A3;
const int ir_arriere = A2;
const float ir_fonction_coeff[5] = { 0.034f, 
                                    -0.424f, 
                                     1.760f, 
                                    -3.127f, 
                                     2.388f};

// Bouton 
const int bouton = 19;

// Batterie
const long batterie_mV_max = 11100; 

// Parametres de filtrage
const int filtre_batterie_decal = 7;
const int filtre_direction_decal = 1;
const int filtre_ir_decal = 4;

// Periodes entre les appels de fonction
const int periode_affichage_ms          = 1000; 
const int periode_lecture_analogique_ms = 10;


/////////////////////////////////////////////////////////////
//
// Types de donnees
//
/////////////////////////////////////////////////////////////

enum direction_t {gauche, centre, droite};
enum mouvement_t {recule, arrete, avance};

/////////////////////////////////////////////////////////////
//
// Variables globales
//
/////////////////////////////////////////////////////////////

// Encodeur de roue
volatile long phase = 15;
volatile byte portd = (PIND & 0x0C) >> 2;

// Etat des filtres
long batterie_filtre = 0;
long direction_filtre = 0;
long ir_avant_filtre = 0;
long ir_arriere_filtre = 0;

// Controle de la vitesse
long vitesse_tic_par_cycle = 0;
long roue_phase_precedente = 0;

// Sequenceur de fonctions
Timer sequenceur;

/////////////////////////////////////////////////////////////
//
// Prototypes
//
/////////////////////////////////////////////////////////////

// Lecture hors interruption de la phase de l'encodeur
long lire_phase();

// Using direct port manipulation for quadrature decoding.
void interruption_encodeur();

// Lecture des entrees analogiques et mise a jour des filtres
void lire_entrees_analogiques();

// Affichage de l'etat du systeme
void affichage();

/////////////////////////////////////////////////////////////
//
// Initialisation
//
/////////////////////////////////////////////////////////////

void setup() {
  // Initialisation de la communication serie
  Serial.begin(115200);
  Serial.print(F("Norbert - Test de la navigation"));
  Serial.println(F("David Beaudette (2017)"));
      
  // Entrees analogiques
  pinMode(mesure_direction, INPUT); 
  pinMode(ir_avant,   INPUT); 
  pinMode(ir_arriere, INPUT); 
  pinMode(batterie, INPUT); 
  analogReference(EXTERNAL);

  // Entrees numeriques
  pinMode(bouton, INPUT_PULLUP); 

  // Initialisation de l'encodeur de roue
  pinMode(encodeur_a, INPUT_PULLUP);
  pinMode(encodeur_b, INPUT_PULLUP);
  attachInterrupt(encodeur_int_a, interruption_encodeur, CHANGE); 
  attachInterrupt(encodeur_int_b, interruption_encodeur, CHANGE);
  
  // Initialisation des fonctions periodiques
  sequenceur.every(periode_lecture_analogique_ms, 
                   lire_entrees_analogiques); 
                   
  sequenceur.every(periode_affichage_ms, 
                   affichage);  
                   
}

/////////////////////////////////////////////////////////////
//
// Programme principal
//
/////////////////////////////////////////////////////////////

void loop() {

  sequenceur.update();
}

// Lecture hors interruption de la phase de l'encodeur
long lire_phase()
{
  noInterrupts();
  long phase_lue = -phase;
  interrupts();
  
  return phase_lue;
}

// Using direct port manipulation for quadrature decoding.
void interruption_encodeur()
{
  byte pd = (PIND & 0x0C) >> 2;
  pd ^= (pd >> 1);     // convert quadrature/Gray to 2 bit binary
  byte sig = (pd - portd) & 3;  // calc difference from last sample
  if(sig == 1)
    --phase;
  else if(sig == 3)
    ++phase;
  portd = pd;
}

// Lecture et filtrage des entrees analogiques
void lire_entrees_analogiques() {
  // Mise a jour des filtres
  batterie_filtre += (analogRead(batterie) - batterie_filtre) >> filtre_batterie_decal;
  ir_avant_filtre += (analogRead(ir_avant) - ir_avant_filtre) >> filtre_ir_decal;
  ir_arriere_filtre += (analogRead(ir_arriere) - ir_arriere_filtre) >> filtre_ir_decal;
  direction_filtre = (analogRead(mesure_direction));// - direction_filtre) >> filtre_direction_decal;
}

void affichage() {
  
#if DEBOGGAGE
  Serial.println(); 
  Serial.print(F("La phase de la roue droite est: ")); 
  long roue_phase_courante = lire_phase();
  vitesse_tic_par_cycle = roue_phase_courante - roue_phase_precedente;
  roue_phase_precedente = roue_phase_courante;
  Serial.println(roue_phase_courante); 
  Serial.print(F("La vitesse est: ")); 
  Serial.println(vitesse_tic_par_cycle); 
  
  Serial.print(F("L'etat de la batterie est: ")); 
  Serial.print(static_cast<float>(batterie_filtre) * 0.0125f);
  Serial.print(F(" V (")); 
  Serial.print(batterie_filtre);
  Serial.println(F(").")); 
  
  float ir_distance;
  float a;
  Serial.print(F("L'etat du capteur avant est: ")); 
  a = static_cast<float>(ir_avant_filtre) * adc_to_volts;
  ir_distance =  ir_fonction_coeff[0] * a * a * a * a;
  ir_distance += ir_fonction_coeff[1] * a * a * a; 
  ir_distance += ir_fonction_coeff[2] * a * a; 
  ir_distance += ir_fonction_coeff[3] * a; 
  ir_distance += ir_fonction_coeff[4]; 
  Serial.print(ir_distance * 100.0f);
  Serial.print(F(" cm (")); 
  Serial.print(ir_avant_filtre);
  Serial.println(F(").")); 
  
  Serial.print(F("L'etat du capteur arriere est: ")); 
  a = static_cast<float>(ir_arriere_filtre) * adc_to_volts;
  ir_distance =  ir_fonction_coeff[0] * a * a * a * a;
  ir_distance += ir_fonction_coeff[1] * a * a * a; 
  ir_distance += ir_fonction_coeff[2] * a * a; 
  ir_distance += ir_fonction_coeff[3] * a; 
  ir_distance += ir_fonction_coeff[4]; 
  Serial.print(ir_distance * 100.0f);
  Serial.print(F(" cm (")); 
  Serial.print(ir_arriere_filtre);
  Serial.println(F(").")); 

  Serial.print(F("Direction lue: "));
  Serial.println(direction_filtre);

#endif

}


