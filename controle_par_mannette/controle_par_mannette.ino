/////////////////////////////////////////////////////////////
//
// norbert-base -> controle_par_manette
//
// Permet de contrôler la base Tonka de Norbert en utilisant
// une mannette fabriquee en Lego
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

// Numero de pin des sorties vers les moteurs
const int moteur_avant   =  9;
const int moteur_arriere = 10;
const int moteur_gauche  = 11;  
const int moteur_droite  =  6; 

// Numero de pin de l'entree de direction
const int mesure_direction = A6;

// Numero de pin de la mannette
const int mannette_avant   =  4;
const int mannette_arriere =  7;
const int mannette_gauche  =  8; 
const int mannette_droite  = 12;  
const int mannette_pot     = A7;  
const int mannette_del     =  5;

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
const int filtre_mannette_pot_decal = 1;

// Parametres de controle des moteurs
const int pwm_max = 255;

const int vitesse_av_moteur_nom = int(0.2f * 255.0f);
const int vitesse_zero_tol_tic_par_cycle = 3;
const float moteur_av_gain_pid[3] = {0.05f, 0.01f, 0.0f};
const int moteur_av_zone_morte = 0;

const float moteur_gd_gain_pid[3] = {0.20f, 0.0f, 0.05f};
const int moteur_gd_zone_morte = 50;
const int moteur_gd_vitesse_max = 150;

// Periodes entre les appels de fonction
const int periode_affichage_ms          = 1000; 
const int periode_controle_moteurs_ms   = 10;
const int periode_lecture_analogique_ms = 5;
const int periode_controle_led_ms       = 10;


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

// Batterie
long batterie_filtre = 0;
long direction_filtre = 0;
long ir_avant_filtre = 0;
long ir_arriere_filtre = 0;
long mannette_pot_filtre = 0;

// Controle de la vitesse
long vitesse_tic_par_cycle = 0;
int vitesse_av_cmd;
long roue_phase_precedente = 0;
long moteur_integrale_erreur = 0;
long moteur_erreur_prec = 0;
int  moteur_commande_prec = 0;

// Controle de la direction
int vitesse_gd_cmd = 0;
long moteur_gd_integrale_erreur = 0;
long moteur_gd_erreur_prec = 0;
int moteur_gd_commande_prec = 0;

// État de la mannette
direction_t interrupteur_mannette_precedent = centre; 
mouvement_t mannette_mouvement_precedent  = arrete; 
bool interrupteur_mannette_modifiee = false; 

// Communication avec le cerveau RPi
//CmdMessenger cerveau = CmdMessenger(Serial, ',',';','/');

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

// Controle de la led sur la manette
void allume_led();
void eteint_led();
void controle_led();

// Controle de la propulsion
void controle_moteurs();

// Controle de la direction
void controle_direction();

// Controle de la direction et de la propulsion
int pid(long &integrale,
        long &erreur_prec,
        int &commande_prec,
        const long commande_desiree, 
        const long lecture,
        const int integrale_erreur_min, 
        const int commande_min, 
        const int commande_max, 
        const float *gain_pid);
        
/////////////////////////////////////////////////////////////
//
// Initialisation
//
/////////////////////////////////////////////////////////////

void setup() {
  // Initialisation de la communication serie
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
  
  // Initialisation des entrees de la mannette
  pinMode(mannette_avant,   INPUT_PULLUP);  
  pinMode(mannette_arriere, INPUT_PULLUP); 
  pinMode(mannette_gauche,  INPUT_PULLUP); 
  pinMode(mannette_droite,  INPUT_PULLUP); 
  
  // Affichage sur la manette
  pinMode(mannette_del,   OUTPUT); 
  
  // Entrees analogiques
  pinMode(mannette_pot, INPUT); 
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
  
  // Initialisation du PID de la direction
  
  // Initialisation des fonctions periodiques
  sequenceur.every(periode_lecture_analogique_ms, 
                   lire_entrees_analogiques); 
                   
  sequenceur.every(periode_affichage_ms, 
                   affichage);  
                   
  sequenceur.every(periode_controle_moteurs_ms, 
                   controle_moteurs);  
                   
  sequenceur.every(periode_controle_moteurs_ms, 
                   controle_direction);  
                   
  sequenceur.every(periode_controle_led_ms, 
                   controle_led);  
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
  mannette_pot_filtre += (analogRead(mannette_pot) - mannette_pot_filtre) >> filtre_mannette_pot_decal;
  direction_filtre = (analogRead(mesure_direction));// - direction_filtre) >> filtre_direction_decal;
 
}

void affichage() {
  
#if DEBOGGAGE
  Serial.println(); 
  Serial.print(F("Mannette mouvement: ")); 
  switch(mannette_mouvement_precedent)
  {
      case recule: Serial.println(F("recule.")); break;
      case arrete: Serial.println(F("arrete.")); break;
      case avance: Serial.println(F("avance.")); break;
  }  
  Serial.print(F("Mannette direction: ")); 
  switch(interrupteur_mannette_precedent)
  {
      case gauche: Serial.println(F("gauche.")); break;
      case centre: Serial.println(F("centre.")); break;
      case droite: Serial.println(F("droite.")); break;
  }  
  Serial.print(F("La phase de la roue droite est: ")); 
  Serial.println(lire_phase()); 
  
  Serial.print(F("La vitesse est: ")); 
  Serial.println(vitesse_tic_par_cycle); 
  
  Serial.print(F("La vitesse commandee est: ")); 
  Serial.println(vitesse_av_cmd); 
  
  Serial.print(F("L'etat du bouton est: ")); 
  Serial.println(digitalRead(bouton));
  
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

  Serial.print(F("L'etat du potentiometre de la mannette est: ")); 
  Serial.println(mannette_pot_filtre);
  
  Serial.print(F("Direction lue: "));
  Serial.println(direction_filtre);

  Serial.print(F("La vitesse commandee est: ")); 
  Serial.println(vitesse_gd_cmd); 
  
#endif

}

void lecture_interrupteur_mannette() {
  
  // Lire l'etat des controles sur la manette
  int interrupteur_gauche = !digitalRead(mannette_gauche);
  int interrupteur_droite = !digitalRead(mannette_droite);
  
  direction_t interrupteur_mannette_actuel;
  
  if(interrupteur_gauche) {
    interrupteur_mannette_actuel = gauche;
  }
  else if(interrupteur_droite) {
    interrupteur_mannette_actuel = droite;
  }
  else {
    interrupteur_mannette_actuel = centre;
  }
  
  // Mise a jour de la memoire de direction
  if(interrupteur_mannette_precedent == interrupteur_mannette_actuel) {
    interrupteur_mannette_modifiee = false;
  }
  else {
    interrupteur_mannette_modifiee = true;
  }
  interrupteur_mannette_precedent = interrupteur_mannette_actuel;
}
  
void controle_direction() {
  
  int direction_cmd = mannette_pot_filtre;
  
  vitesse_gd_cmd = pid(moteur_gd_integrale_erreur, 
                       moteur_gd_erreur_prec,
                       moteur_gd_commande_prec,
                       direction_cmd,
                       direction_filtre,
                      -moteur_gd_vitesse_max, 
                       moteur_gd_vitesse_max,
                       moteur_gd_gain_pid);
  
  if(vitesse_gd_cmd > moteur_gd_zone_morte &&
     direction_filtre < 1020) {
    analogWrite(moteur_gauche, vitesse_gd_cmd);    
    analogWrite(moteur_droite, 0);    
  }
  else if(vitesse_gd_cmd < -moteur_gd_zone_morte &&
          direction_filtre > 3) {
    analogWrite(moteur_gauche, 0);    
    analogWrite(moteur_droite, -vitesse_gd_cmd);    
  }
  else {
    analogWrite(moteur_gauche, 0);    
    analogWrite(moteur_droite, 0);    
  }
}  

void allume_led() {
  digitalWrite(mannette_del, 1);
}

void eteint_led() {
  digitalWrite(mannette_del, 0);
}

void controle_led() {
  
  switch(interrupteur_mannette_precedent)
  {
    case gauche: 
      allume_led(); 
      break;
    case centre: 
      if(interrupteur_mannette_modifiee) {
        sequenceur.oscillate(mannette_del, 1000U, 1, 3);
      }
      break;
    case droite: 
      eteint_led(); 
      break;
  }  
}  

void controle_moteurs() {
  
  // Lire l'etat des controles sur la manette
  int commande_avance = !digitalRead(mannette_avant);
  int commande_recule = !digitalRead(mannette_arriere);
  
  mouvement_t mannette_mouvement_actuel;
  
  if(commande_avance) {
    mannette_mouvement_actuel = avance;
  }
  else if(commande_recule) {
    mannette_mouvement_actuel = recule;
  }
  else {
    mannette_mouvement_actuel = arrete;
  }
   
  // Lire la phase de la roue
  long roue_phase_courante = lire_phase();
  vitesse_tic_par_cycle = roue_phase_courante - roue_phase_precedente;
  
  if(digitalRead(bouton) == LOW) {
    // Vitesse basee sur la distance arriere
    vitesse_av_cmd = map(ir_arriere_filtre, 184, 850, 20, pwm_max);
  }
  else if(mannette_mouvement_actuel == arrete) {
    // Freinage
    vitesse_av_cmd = pid(moteur_integrale_erreur, 
                         moteur_erreur_prec,
                         moteur_commande_prec,
                         0,
                         vitesse_tic_par_cycle,
                        -pwm_max, 
                         pwm_max,
                         moteur_av_gain_pid);
  }
  else {   
    if(mannette_mouvement_actuel == avance) {
      // Activer le moteur avant
       vitesse_av_cmd = vitesse_av_moteur_nom;
    }
    else if(mannette_mouvement_actuel == recule) {
      // Activer le moteur arrière
       vitesse_av_cmd = -vitesse_av_moteur_nom;
    }
  }
  
  // Mise a jour de la commande PWM
  if(vitesse_av_cmd > moteur_av_zone_morte) {
    // Activer le moteur avant
    analogWrite(moteur_avant, vitesse_av_cmd);
    analogWrite(moteur_arriere, 0);
  }
  else if(vitesse_av_cmd < -moteur_av_zone_morte) {
    // Activer le moteur arrière
    analogWrite(moteur_avant, 0);
    analogWrite(moteur_arriere, -vitesse_av_cmd);
  }
  else {    
    analogWrite(moteur_avant, 0);
    analogWrite(moteur_arriere, 0);
  }
  // Mise a jour de la memoire
  mannette_mouvement_precedent = mannette_mouvement_actuel;
  roue_phase_precedente = roue_phase_courante;
}  

int pid(long &integrale,
        long &erreur_prec,
        int &commande_prec,
        const long commande_desiree, 
        const long lecture,
        const int commande_min, 
        const int commande_max, 
        const float *gain_pid) {
  
  long erreur = commande_desiree - lecture;
  
  float commande_pid = gain_pid[0] * (float)erreur + 
                       gain_pid[1] * (float)(erreur - erreur_prec) + 
                       gain_pid[2] * (float)integrale;
    
  int commande_actuelle;
  int commande_non_saturee = static_cast<int>(commande_pid);
  
  bool saturation = true;
  if(commande_non_saturee < commande_min) 
    commande_actuelle = commande_min;
  else if(commande_non_saturee > commande_max) 
    commande_actuelle = commande_max;
  else {
    commande_actuelle = commande_non_saturee;
    saturation = false;
  }
  
  // L'integrale diverge sans mise a jour conditionnelle
  if(erreur * commande_non_saturee <= 0 || 
     !saturation) {
    integrale += erreur;      
  }
  
  // Mise a jour de l'erreur        
  erreur_prec = erreur;
  commande_prec = commande_non_saturee;
  return commande_actuelle;        
}
