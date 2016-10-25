/////////////////////////////////////////////////////////////
//
// norbert-base -> controle_par_manette
//
// Permet de contrôler la base Tonka de Norbert en utilisant
// une mannette fabriquée en Lego
//
// David Beaudette (2016)
//
/////////////////////////////////////////////////////////////

//#include <CmdMessenger.h>
#include <Timer.h>
#include <Event.h>

/////////////////////////////////////////////////////////////
//
// Constantes
//
/////////////////////////////////////////////////////////////

#define DEBOGGAGE 0

// Encodeur de roue droite
const int encodeur_a = 2;
const int encodeur_b = 3;
const int encodeur_int_a = 0;
const int encodeur_int_b = 1;

// Numéro de pin des sorties vers les moteurs
const int moteur_avant   =  9;
const int moteur_arriere = 10;
const int moteur_gauche  =  6;  
const int moteur_droite  = 11; 

// Numéro de pin de l'entrée de direction
const int mesure_direction = A6;

// Numéro de pin de la mannette
const int mannette_avant   =  4;
const int mannette_arriere =  7;
const int mannette_gauche  =  8; 
const int mannette_droite  = 12;  
const int mannette_pot     = A7;  
const int mannette_del     =  5;

// Batterie
const int batterie = A4;

// Capteurs de distance
const int ir_avant   = A3;
const int ir_arriere = A2;

// Bouton 
const int bouton = 19;

// Batterie
const long batterie_mV_max = 11100; 

// Parametres de filtrage
const int filtre_batterie_decal = 7;
const int filtre_direction_decal = 3;
const int filtre_ir_decal = 4;
const int filtre_mannette_pot_decal = 5;

// Parametres de controle des moteurs
const int vitesse_zero_tol_tic_par_cycle = 3;
const float moteur_gain_pid[3] = {0.05f, 0.01f, 0.0f};

// Périodes entre les appels de fonction
const int periode_affichage_ms = 1000; 
const int periode_lecture_analogique_ms = 10;
const int periode_controle_moteurs_ms = 100;


/////////////////////////////////////////////////////////////
//
// Variables globales
//
/////////////////////////////////////////////////////////////

// Encodeur de roue
volatile long phase = 15;
volatile byte portd = (PIND & 0x0C) >> 2;

// Batterie
long batterie_filtre = 0;
long direction_filtre = 0;
long ir_avant_filtre = 0;
long ir_arriere_filtre = 0;
long mannette_pot_filtre = 0;

// Controle de la vitesse
long vitesse_tic_par_cycle = 0;
long vitesse_cmd;
long roue_phase_precedente = 0;
long moteur_integrale_erreur = 0;
long moteur_erreur_prec = 0;

// Communication avec le cerveau RPi
//CmdMessenger cerveau = CmdMessenger(Serial, ',',';','/');

// Séquenceur de fonctions
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

// Lecture des entrées analogiques et mise a jour des filtres
void lire_entrees_analogiques();

// Affichage de l'état du systeme
void affichage();

// Controle de la direction et de la propulsion
void controle_moteurs();

// Controle de la direction et de la propulsion
float pid(long &integrale,
          long &erreur_prec,
          const long commande_desiree, 
          const long lecture, 
          const float *gain_pid);

/////////////////////////////////////////////////////////////
//
// Initialisation
//
/////////////////////////////////////////////////////////////

void setup() {
  // Initialisation de la communication série
  Serial.begin(115200);
  char o_accent_circonflexe = 244;
  Serial.print(F("Contr"));
  Serial.print(o_accent_circonflexe);
  Serial.println(F("le de Norbert par mannette"));
  Serial.println(F("David Beaudette (2016)"));
  
  // Initialisation des sorties vers les moteurs
  pinMode(moteur_avant,   OUTPUT);  
  pinMode(moteur_arriere, OUTPUT);
  pinMode(moteur_gauche,  OUTPUT);  
  pinMode(moteur_droite,  OUTPUT); 
  
  // Initialisation des entrées de la mannette
  pinMode(mannette_avant,   INPUT_PULLUP);  
  pinMode(mannette_arriere, INPUT_PULLUP); 
  pinMode(mannette_gauche,  INPUT_PULLUP); 
  pinMode(mannette_droite,  INPUT_PULLUP); 
  
  // Entrées analogiques
  pinMode(mannette_pot, INPUT); 
  pinMode(mesure_direction, INPUT); 
  pinMode(ir_avant,   INPUT); 
  pinMode(ir_arriere, INPUT); 
  pinMode(batterie, INPUT); 
  analogReference(EXTERNAL);

  // Entrées numériques
  pinMode(bouton, INPUT_PULLUP); 

  // Initialisation de l'encodeur de roue
  pinMode(encodeur_a, INPUT_PULLUP);
  pinMode(encodeur_b, INPUT_PULLUP);
  attachInterrupt(encodeur_int_a, interruption_encodeur, CHANGE); 
  attachInterrupt(encodeur_int_b, interruption_encodeur, CHANGE);
  
  // Initialisation du PID de la direction
  
  // Initialisation des fonctions périodiques
  sequenceur.every(periode_lecture_analogique_ms, 
                   lire_entrees_analogiques); 
                   
  sequenceur.every(periode_affichage_ms, 
                   affichage);  
                   
  sequenceur.every(periode_controle_moteurs_ms, 
                   controle_moteurs);  
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

// Lecture et filtrage des entrées analogiques
void lire_entrees_analogiques() {
  // Mise a jour des filtres
  batterie_filtre += (analogRead(batterie) - batterie_filtre) >> filtre_batterie_decal;
  ir_avant_filtre += (analogRead(ir_avant) - ir_avant_filtre) >> filtre_ir_decal;
  ir_arriere_filtre += (analogRead(ir_arriere) - ir_arriere_filtre) >> filtre_ir_decal;
  mannette_pot_filtre += (analogRead(mannette_pot) - mannette_pot_filtre) >> filtre_mannette_pot_decal;
  direction_filtre += (analogRead(mesure_direction) - direction_filtre) >> filtre_direction_decal;
 
}

void affichage() {
  
#if DEBOGGAGE
  Serial.print(F("Mannette avant:")); 
  Serial.println(digitalRead(mannette_avant)); 
  Serial.print(F("Mannettea arriere:")); 
  Serial.println(digitalRead(mannette_arriere)); 
  Serial.print(F("La phase de la roue droite est:")); 
  Serial.println(lire_phase()); 
  Serial.print(F("La vitesse est:")); 
  Serial.println(vitesse_tic_par_cycle); 
  Serial.print(F("La vitesse commandée est:")); 
  Serial.println(vitesse_cmd); 
  Serial.print(F("L'etat du bouton est:")); 
  Serial.println(digitalRead(bouton));
  Serial.print(F("L'etat de la batterie est:")); 
  Serial.println(batterie_filtre);
  Serial.print(F("L'etat du capteur avant est:")); 
  Serial.println(ir_avant_filtre);
  Serial.print(F("L'etat du capteur arriere est:")); 
  Serial.println(ir_arriere_filtre);
  Serial.print(F("L'etat du potentiometre de la mannette est:")); 
  Serial.println(mannette_pot_filtre);
  Serial.print(F("Direction lue: "));
  Serial.println(direction_filtre);

#endif

}

void controle_moteurs() {
  
  // Lire l'état des controles sur la manette
  int commande_avance = digitalRead(mannette_avant);
  int commande_recule = digitalRead(mannette_arriere);
  
  // Lire la phase de la roue
  long roue_phase_courante = lire_phase();
  vitesse_tic_par_cycle = roue_phase_courante - roue_phase_precedente;
  
  // PWM entre 20% et 100%
  int vitesse_moteur_min = int(0.2f * 255.0f);
  int vitesse_moteur_nom = int(0.2f * 255.0f);
  if(commande_avance == HIGH && commande_recule == HIGH) {
    // Freinage
    vitesse_cmd = constrain((int)pid(moteur_integrale_erreur, 
                                     moteur_erreur_prec,
                                     0,
                                     vitesse_tic_par_cycle,
                                     moteur_gain_pid), -255, 255);
    if(abs(vitesse_cmd) < vitesse_moteur_min) vitesse_cmd = 0;
  }
  else {   
    if(commande_avance == LOW) {
      // Activer le moteur avant
       vitesse_cmd = vitesse_moteur_nom;
    }
    else if(commande_recule == LOW) {
      // Activer le moteur arrière
       vitesse_cmd = -vitesse_moteur_nom;
    }
  }
  
  // Mise a jour de la commande PWM
  if(vitesse_cmd > 0) {
    // Activer le moteur avant
    analogWrite(moteur_avant, vitesse_cmd);
    analogWrite(moteur_arriere, 0);
  }
  else if(vitesse_cmd < 0) {
    // Activer le moteur arrière
    analogWrite(moteur_avant, 0);
    analogWrite(moteur_arriere, -vitesse_cmd);
  }
  else {    
    analogWrite(moteur_avant, 0);
    analogWrite(moteur_arriere, 0);
  }
  // Mise a jour de la phase précédente
  roue_phase_precedente = roue_phase_courante;
}  

float pid(long &integrale,
          long &erreur_prec,
          const long commande_desiree, 
          const long lecture, 
          const float *gain_pid) {
  
  long erreur = commande_desiree - lecture;
  
  // A voir si l'intégrale diverge sans mise a jour conditionnelle
  integrale += erreur;      
          
  float commande_pid = gain_pid[0] * (float)erreur + 
                       gain_pid[1] * (float)(erreur - erreur_prec) + 
                       gain_pid[2] * (float)integrale;
    
  // Mise a jour de l'erreur        
  erreur_prec = erreur;
  
  return commande_pid;        
}
