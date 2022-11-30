#include <Arduino.h>
#include <librobus.h>
#include <math.h>

// ======================== Définition compilation conditionnel =========================
//#define DEBUGGING_DISTANCE_ROB_POINT
//#define DEBUGGING_CARTESIEN
//#define TUNNING_PID_Theta
//#define TUNNING_PID_VR
//#define DEBUGGING
//#define DEBUGGING_CAPTEURS
//#define PAS_DE_MOUVEMENT
//#define LECTURE_ORRIENTATION_DEPART
//#define LOOP_DE_TEST
//#define TEST_ROUGE
//#define TEST_JAUNE
//#define TEST_VERT
//#define TEST_BLEU
//#define DEUXIEME_TOUR
//#define ETAT_BRAS
//#define TEST_SANS_BRAS
//#define TEST_COM

#include "ComMarketing.h"
//================================================================================================================================
//                                              Définitions des macros (termes) constants
//================================================================================================================================

const float LEN_JUMP = 79;                        // Longueur du jump [mm]
const float HEI_JUMP = 19;                        // Hauteur du jump [mm]
const float HYP_JUMP = hypot(LEN_JUMP, HEI_JUMP); // Hypothénuse du jump [mm]

const int WHEEL_DIA_IN = 3;                 // Diamètre des roues [po]
const float IN_2_MM = 25.4;                 // Constante de conversion [po] -> [mm]
const float r = WHEEL_DIA_IN * IN_2_MM / 2; // Rayon des roues [mm]
const float C = 2 * PI * r;                 // Circonférence des roues [mm]
const int Q = 3200;                         // Nombre de pulses lus par les encodeurs pour une révolution des roues
const float TICKS_PER_MM = Q / C;           // Nombre de pulses lus par les encodeurs pour un centimètre
const float MM_PER_TICKS = C / Q;           // Nombre de centimètres par pulses
const float L = 186.5;                      // Distance entre les deux roues [mm]

/*****************PID*********************
const float KP1 = 0.0001;
const float KI1 = 0.00001;
const float KP2 = 0.001;
*/

#define FIRST_SECOND_POS_X 0                  // Coordonnée longitudinale première vague [cm]
#define THIRD_FOURTH_POS_X 0                  // Coordonnée longitudinale deuxième vague [cm]
#define FIFTH_SIXTH_POS_X 0                   // Coordonnée longitudinale troisième vague [cm]
#define SEVENTH_EIGHT_POS_X 0                 // Coordonnée longitudinale quatrième vague [cm]
#define FIRST_THIRD_FIFTH_SEVENTH_POS_Y 76.2  // Coordonnée transversale ligne verte [cm]
#define SECOND_FOURTH_SIXTH_EIGHT_POS_Y 45.72 // Coordonnée transversale ligne jaune [cm]

#define DIA_WHEEL_IN 3                            // Diamètre des roues [po]
#define IN_2_CM 2.54                              // Constante de conversion [po] -> [cm]
#define RADIUS_WHEEL_CM DIA_WHEEL_IN *IN_2_CM / 2 // Rayon des roues [cm]
#define C DIA_WHEEL_IN *IN_2_CM *PI               // Circonférence des roues [cm]
#define Q 3200                                    // Nombre de pulses lus par les encodeurs pour une révolution des roues
#define TICKS_PER_CM Q / C                        // Nombre de pulses lus par les encodeurs pour un centimètre
#define CM_PER_TICKS C / Q                        // Nombre de centimètre par pulses
#define R_RW 0                                    // Distance centre RobUS -> centre roue droite [cm]
#define R_LW 0                                    // Distance centre RobUS -> centre roue gauche [cm]
#define TRACK_WIDTH 18.7
#define TICKS_TRACK_WIDTH TICKS_PER_CM *TRACK_WIDTH
#define RADIUS_WHEEL_TICKS RADIUS_WHEEL_CM *TICKS_PER_CM // Rayon des roues [cm]

//=========LED COULEUR=========

#define BLUE 1   // Valeur associée à la couleur bleu (utilisée dans les qualifications)
#define GREEN 2  // Valeur associée à la couleur vert (utilisée dans les qualifications)
#define YELLOW 3 // Valeur associée à la couleur jaune (utilisée dans les qualifications)
#define RED 4    // Valeur associée à la couleur rouge (utilisée dans les qualifications)

#define LED_BLUE 48
#define LED_GREEN 49
#define LED_YELLOW 46
#define LED_RED 47

//=======Suiveur de ligne============
#define LIGNE_1 39 // pins du suiveur de ligne
#define LIGNE_2 38
#define LIGNE_3 41
#define LIGNE_4 40
#define LIGNE_5 43
#define LIGNE_6 42
#define LIGNE_7 45
#define LIGNE_8 44
bool ligne[8];

//*****************PID**********************

#define KP1 10      // 5 ou 10 ou 15
#define KI1 0.00001 // 0.00001 ou 0.001
#define KP2 0.005   // 0.01 ou 0.005
#define KI2 0.00005 // 0.0001 ou 0.00005
/*à 0.2 CV.vr
#define KP1 10      // 5 ou 10 ou 15
#define KI1 0.00001 // 0.00001 ou 0.001
#define KP2 0.005   // 0.01 ou 0.005
#define KI2 0.00005 // 0.0001 ou 0.00005

à 0.35 pas fini
#define KP1 5      // 5 ou 10 ou 15
#define KI1 0.001 // 0.00001 ou 0.001
#define KP2 0.005   // 0.01 ou 0.005
#define KI2 0.0001 // 0.0001 ou 0.00005
*/
//================================================================================================================================
//                                                     Définitions des variables
//================================================================================================================================

/**************** Cartésien ****************/

// Compteurs de temps écoulé [ms] et intervalle de temps [ms] entre 2 mesures
unsigned long myTime = millis();
unsigned long elapsedTime = 0;

// Plan cartésien
struct VariablesCartesien
{
  float dL; // Distance parcourue par la roue droite [cm]
  float dR; // Distance parcourue par la roue gauche [cm]

  float dC; // Distance parcourue par le robot [cm]

  float theta;  // Orientation du robot [rad]
  float Dtheta; // Différentielle de l'orientation du robot [rad]
  float Dx;     // Position horizontale relative du robot [cm]
  float Dy;     // Position transversale relative du robot [cm]

  // Coordonnées initales du robot
  float coordx;
  float coordy;

  // Vitesses linéaires et vitesse angulaire
  float vx, vy, vL, vR, v, w;
};

struct VariablesCartesien Cart = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//========== variables void loop===========

int point_i = 0;        // Itérateur des points de la matrice du trajet
float rCompletion = 60; // Rayon autour d'un point pour être considéré atteint [mm]

//===== variables changement de trajet=======
int trajet_precedant = 0;

/*
// PID2M2T
float ephiIntegral = 0; // Erreur intégrale de l'orientation [rad]
float ephiPrevious = 0; // Erreur précédente de l'orientation [rad]
*/
/*
// Qualifications
int orientationInit = 0;  // Valeur du dé pour orientation initiale -> bumper avant
float phiInit = 0;  // Orientation initiale du robot [rad]
int posQuille1 = 0; // Valeur du dé pour la position globale de la quille 1 (zone 2 = 1, zone 3 = 2, zone 4 = 3, zone 5 = 4, zone 6 = 5, zone 8 = 6) -> bumper droit
int posQuille2 = 0; // Valeur du dé pour la position globale de la quille 2 (zone 2 = 1, zone 3 = 2, zone 4 = 3, zone 5 = 4, zone 6 = 5, zone 8 = 6) -> bumper gauche
int countQuille = 0;  // Compteur du nombre de quilles renversées (0, 1 ou 2)
*/

float Spirale[][2] = {{0.0, -0.0}, {-32.18331358525289, 78.27947394830758}, {-52.17526813848373, 160.49218536080897}, {-60.026221494162776, 244.81673508777698}, {-55.78653148676028, 329.43172397948445}, {-39.50655595074651, 412.5157528862041}, {-11.23665272059176, 492.2474226582087}, {28.94570230688324, 566.7763562465426}, {80.16534309188258, 634.1116813421103}, {140.85890137725735, 693.0935669757914}, {209.4741779614504, 742.767702393527}, {284.45897364290414, 782.1797768412582}, {364.26108922006154, 810.3754795649265}, {447.3283254913654, 826.4004998104731}, {531.9468972171496, 829.4728488492366}, {616.0656288146997, 819.82958068483}, {697.783528257615, 797.76884921994}, {775.1880603110469, 763.553673341131}, {846.3458972922522, 717.6838132584058}, {909.5445745029905, 661.3575939157948}, {963.1714999449656, 595.9000187966694}, {1005.6200027109777, 522.6755575871723}, {1035.6777688945053, 443.5331866184492}, {1052.6295464308762, 360.6146554260427}, {1055.787025083775, 276.06245194446836}, {1044.6737096131044, 192.1490830355574}, {1019.7238697126195, 111.2626046854152}, {981.6116401941963, 35.72682329289385}, {931.0617337548105, -32.126414370738445}, {869.5458734856918, -90.23960165149263}, {799.0546805693186, -137.0763564963957}, {721.5914126391673, -171.1441230227526}, {639.4147397541666, -191.34083396571148}, {554.9933523342872, -197.34870031821015}, {470.7856936595899, -188.94270411979969}, {389.2824525288888, -166.1819286871019}, {312.77817796415894, -129.98470597776142}, {243.46493783842186, -81.43022733730352}, {183.4819262555486, -21.737803807095727}, {134.41851825428716, 47.22391204525333}, {97.57581602641365, 123.41474474382107}, {74.3327223026461, 204.9132648336102}, {66.10414567465693, 289.4195695474819}, {73.80553887227362, 373.63570617787104}, {98.3133648853013, 454.2193205265763}, {140.23656590125032, 527.6056943165034}, {198.07009488105356, 589.2362392263218}, {269.3312318588124, 634.6604170946028}, {349.9010602210644, 660.0241574884565}, {434.24216917816864, 662.3505195607477}, {516.1013725256546, 641.5967707208756}, {589.9498074335241, 600.3654497254046}, {650.8407654074405, 541.8683228092668}, {696.2203002501692, 470.5051754480156}, {724.2346568553809, 390.6499354885887}, {733.2104202023527, 306.38599367398984}, {722.2436412859669, 222.6231778956697}, {690.5774825339757, 144.5033351483877}, {638.8266384260912, 77.7389065948313}, {570.3375113758226, 28.388571485857668}, {490.29188965458957, 1.6803691432795251}, {405.92531053188463, 1.3327726861799982}, {325.744713820514, 27.557964960261714}, {258.15670790698243, 78.05302809598363}, {210.18362746223178, 147.42549394334972}, {187.15264189998868, 228.58374917902907}, {191.55557443911766, 312.80082084270447}, {223.09440565011525, 391.0360659461832}, {278.5915160417563, 454.54246258844205}, {352.8021560213972, 494.34096742047194}, {436.53804104321773, 502.11584835855615}, {515.5104745419757, 473.47194073524855}, {572.1936713097934, 411.65226198927996}, {590.7706275001742, 330.1167656439043}, {566.480180864213, 249.80577061861734}, {507.553811896467, 189.99363966298577}, {428.2842979585859, 162.5689131924592}, {351.42400656091917, 191.10297763190223}, {341.9308668205066, 269.32342770768395}, {417.16421149544004, 296.4457754583175}, {497.1645307456394, 269.06380957490467}, {576.9086251990508, 240.3371096101764}, {656.9062974573992, 212.32293236799546}, {735.5207157974053, 180.93025593961335}, {811.1360273354418, 141.81846902196853}, {880.0735473923318, 92.21313183189088}, {937.3152736570496, 30.78429373523408}, {977.5320112964502, -43.28303221027134}, {996.0872633626266, -125.69487613809707}, {992.2934888678699, -210.1143947626354}, {965.8309540758129, -290.29782545043213}, {918.0426668905516, -359.90800087668464}, {853.4036358278105, -414.3687689130737}, {776.7079851860863, -449.6889308971518}, {693.2074242233062, -462.22861434386226}, {609.371937394344, -451.64962651121857}, {531.2929051392258, -419.3912333768553}, {465.07027145644474, -367.0100819745866}, {415.9353482708024, -298.3207648052839}, {386.6549735234582, -219.03689743431525}, {379.69097, -134.89021}};

float Cercle[][2] = {{-478.65705, -276.35276}, {-495.0465277382876, -245.74661222100087}, {-509.4639511197873, -214.1656663119307}, {-521.8740791032943, -181.7414440405222}, {-532.2416706476042, -148.6054671745084}, {-540.5314847115126, -114.88925748162224}, {-546.7082802538143, -80.72433672959664}, {-550.7368162333056, -46.24222668616448}, {-552.5818516087813, -11.574449119058764}, {-552.2116259446512, 23.142894008136526}, {-549.6348407613317, 57.7633958322528}, {-544.8863020269807, 92.15438961069154}, {-538.001250782087, 126.18435353733628}, {-529.0149280671397, 159.72176580607064}, {-517.9625749226277, 192.635104610778}, {-504.8794323890399, 224.79284814534185}, {-489.8007415068652, 256.0634746036458}, {-472.76269365534483, 286.3146884519927}, {-453.8309579045044, 315.41534733529704}, {-433.1010886587106, 343.2634631745756}, {-410.6693664961508, 369.76275518428145}, {-386.63207199501164, 394.8169425788676}, {-361.08548573347997, 418.32974457278726}, {-334.1258882897426, 440.2048803804935}, {-305.84956024198664, 460.34606921643933}, {-276.35278216839936, 478.65703029508}, {-245.7466341143984, 495.0465070829673}, {-214.16568796473808, 509.46393019863126}, {-181.7414655028377, 521.8740583948046}, {-148.60548851211675, 532.2416504242194}, {-114.88927877599434, 540.5314650396085}, {-80.72435807789007, 546.708260993704}, {-46.242248201223326, 550.7367970392388}, {-11.574470929413511, 552.581831928945}, {23.142872427879585, 552.2116052406609}, {57.76337489937117, 549.63481914466}, {92.15436937719085, 544.8862797601616}, {126.18433396837192, 538.0012282295476}, {159.72174677994798, 529.0149056951993}, {192.63508591895237, 517.9625532994982}, {224.79282949241866, 504.8794121848257}, {256.06345560738026, 489.80072349356374}, {286.31466842469115, 472.7626764577869}, {315.41532575052423, 453.8309379979069}, {343.2634402471978, 433.10106694776454}, {369.7627312449434, 410.66934376976866}, {394.81691807399204, 386.63204892632774}, {418.3297200645752, 361.0854628798505}, {440.20485654692465, 334.1258660927454}, {460.34604685127135, 305.84953902742114}, {478.6570103078465, 276.3527621462848}, {495.04648653633205, 245.74661210679298}, {509.46390923665894, 214.16566489554694}, {521.8740371653877, 181.74144210285598}, {532.2416290790791, 148.60546531902963}, {540.5314437342937, 114.88925613437712}, {546.7082398875924, 80.72433613920798}, {550.7367762955357, 46.242226923831936}, {552.5818117146844, 11.574450078557598}, {552.2115856896445, -23.142892952219103}, {549.63480022386, -57.763395011932936}, {544.8862613246646, -92.1543890889306}, {538.0012100298097, -126.18435331195934}, {529.0148873770474, -159.7217658097665}, {517.9625344041291, -192.6351047110993}, {504.8793921488066, -224.79284814470503}, {489.8007016488315, -256.063474239331}, {472.7626532237993, -286.31468726307054}, {453.8309154578917, -315.4153450416115}, {433.1010450655324, -343.2634600271823}, {410.66932250912964, -369.7627515500151}, {386.6320282510922, -394.81693894034083}, {361.08544275382866, -418.32974152839085}, {334.1258464797477, -440.2048786443968}, {305.84951989125784, -460.3460696185897}, {276.3527434507672, -478.6570337812003}, {245.74659326483422, -495.04650782630614}, {214.16564583895612, -509.46392904779816}, {181.74142278306556, -521.8740560967234}, {148.60544570709504, -532.241647624129}, {114.88923622097717, -540.5314622810617}, {80.72431593464441, -546.7082587185686}, {46.24220645802967, -550.7367955876962}, {11.574429401064705, -552.5818315394918}, {-23.14291342176741, -552.211605853583}, {-57.76341475904983, -549.6348201025693}, {-92.15440795406086, -544.886280557413}, {-126.18437139651661, -538.0012285670784}, {-159.7217834761331, -529.014905480529}, {-192.63512258262648, -517.9625526467289}, {-224.7928671057128, -504.87941141464205}, {-256.06349543510805, -489.80072313323205}, {-286.3147107498849, -472.76267709695696}, {-315.4153681513412, -453.83093909374094}, {-343.26348219327394, -433.10106785558696}, {-369.76277241327506, -410.669344052263}, {-394.81695834893577, -386.6320483535381}, {-418.32975953784717, -361.08546142918067}, {-440.2048955176008, -334.1258639489593}, {-460.34608582578807, -305.8495365826428}, {-478.65705, -276.35276}};

float Triangle[][2] = {{0.0, -0.0}, {14.359710473209566, 24.871749000007778}, {28.71942094641913, 49.743498000015556}, {43.079131419628695, 74.61524700002333}, {57.43884189283826, 99.48699600003111}, {71.79855236604783, 124.35874500003891}, {86.15826283925739, 149.23049400004666}, {100.51797331246696, 174.10224300005447}, {114.87768378567652, 198.97399200006222}, {129.2373942588861, 223.84574100007003}, {143.59710473209566, 248.71749000007782}, {157.95681520530525, 273.58923900008557}, {172.31652567851478, 298.4609880000933}, {186.67623615172437, 323.3327370001012}, {201.03594662493393, 348.20448600010894}, {215.3956570981435, 373.0762350001167}, {229.75536757135305, 397.94798400012445}, {244.11507804456264, 422.8197330001323}, {258.4747885177722, 447.69148200014007}, {272.83449899098173, 472.56323100014777}, {287.1942094641913, 497.43498000015563}, {301.55391993740085, 522.3067290001634}, {315.9136304106105, 547.1784780001711}, {330.27334088382, 572.0502270001789}, {344.63305135702956, 596.9219760001866}, {358.99276183023915, 621.7937250001945}, {373.35247230344874, 646.6654740002024}, {387.71218277665827, 671.53722300021}, {402.07189324986786, 696.4089720002179}, {416.4316037230774, 721.2807210002255}, {430.791314196287, 746.1524700002334}, {445.1510246694965, 771.0242190002411}, {459.5107351427061, 795.8959680002489}, {473.8704456159157, 820.7677170002568}, {488.23015668912564, 812.477114346146}, {502.58986806233514, 787.6053658657535}, {516.9495794355447, 762.733617385361}, {531.3092908087542, 737.8618689049684}, {545.6690021819638, 712.9901204245758}, {560.0287135551735, 688.1183719441833}, {574.388424928383, 663.2466234637908}, {588.7481363015925, 638.3748749833984}, {603.1078476748021, 613.5031265030058}, {617.4675590480116, 588.6313780226133}, {631.8272704212212, 563.7596295422206}, {646.1869817944307, 538.8878810618282}, {660.5466931676403, 514.0161325814356}, {674.9064045408498, 489.14438410104316}, {689.2661159140594, 464.27263562065065}, {703.625827287269, 439.4008871402581}, {717.9855386604786, 414.5291386598655}, {732.3452500336881, 389.657390179473}, {746.7049614068976, 364.7856416990805}, {761.0646727801072, 339.91389321868786}, {775.4243841533169, 315.0421447382953}, {789.7840955265265, 290.1703962579028}, {804.143806899736, 265.29864777751027}, {818.5035182729455, 240.42689929711798}, {832.863229646155, 215.55515081672547}, {847.2229410193645, 190.68340233633285}, {861.5826523925741, 165.81165385594034}, {875.9423637657837, 140.9399053755477}, {890.3020751389932, 116.0681568951552}, {904.6617865122028, 91.19640841476269}, {919.0214978854124, 66.32465993437006}, {933.381209258622, 41.45291145397755}, {947.7409206318316, 16.581162973585037}, {947.7409163363371, -0.0}, {919.0214946291753, -0.0}, {890.3020729220138, -0.0}, {861.5826512148521, -0.0}, {832.8632295076903, -0.0}, {804.1438078005286, -0.0}, {775.4243860933668, -0.0}, {746.7049643862051, -0.0}, {717.9855426790433, -0.0}, {689.2661209718816, -0.0}, {660.5466992647198, -0.0}, {631.8272775575581, -0.0}, {603.1078558503963, -0.0}, {574.3884341432345, -0.0}, {545.6690124360728, -0.0}, {516.9495907289113, -0.0}, {488.23016902174953, -0.0}, {459.5107473145879, -0.0}, {430.7913256074261, -0.0}, {402.07190390026426, -0.0}, {373.35248219310256, -0.0}, {344.63306048594075, -0.0}, {315.91363877877905, -0.0}, {287.19421707161723, -0.0}, {258.47479536445553, -0.0}, {229.75537365729372, -0.0}, {201.03595195013202, -0.0}, {172.31653024297054, -0.0}, {143.59710853580884, -0.0}, {114.87768682864703, -0.0}, {86.15826512148521, -0.0}, {57.438843414323514, -0.0}, {28.7194217071617, -0.0}, {0.0, -0.0}};

float Megagenial[][2]={{0.0,0.0},{2000.0,0.0}}; //2 ou 3 points

float (*ptrPoint)[2];

//==========Variables PID==========
struct VariablesPID
{

  float Dx;
  float Dy;
  float SP1;
  float eTheta_KP;
  float eTheta_KI;
  float CV_w;
  float CV_vl;

  float SP2;
  float eVr_KP;
  float eVr_KI;
  float CV_vr;
};

struct VariablesPID PID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//==========Variables detection quille========
unsigned long Bras_TOFF = 0;
bool Bras_state = 0;

//===========Variables distance=============
float calcul_preced = 0;

//============Variables capteur couleur================
/*RANGE RGB SELON LA COULEUR DÉTECTÉE


rouge → r=87-90 g=80-87 b=85-88

jaune → r=94-96 g=93-94 b=67-71

vert → r=61-65 g=96-98 b=94-96

bleu → r=57-61 g=89-92 b=110-113

*/

float speed = 0.2;
bool findHuman = 0;
unsigned long myMouvmentTime = millis();
unsigned long myMouvmentDelay = 0;
int rCompletion_decroisant = 0;

//================================================================================================================================
//                                                     Déclaration des fonctions
//================================================================================================================================

float *cartesien(); // Positionnement global (coordonnées et orientation)
void pid(float *);  // PID 1 moteur avec erreur sur l'orientation

void depart();
void deplacement();
void danse();
void danse2();
void danse3();

void getLigne(bool dLigne[8]);

//================================================================================================================================
//                                                         Programmes Arduino
//================================================================================================================================

void setup()
{
  BoardInit();          // Initialisation de librobus
  Serial.begin(9600); // Modification du Baud Rate
  
  pinMode(LIGNE_1, INPUT);
  pinMode(LIGNE_2, INPUT);
  pinMode(LIGNE_3, INPUT);
  pinMode(LIGNE_4, INPUT);
  pinMode(LIGNE_5, INPUT);
  pinMode(LIGNE_6, INPUT);
  pinMode(LIGNE_7, INPUT);
  pinMode(LIGNE_8, INPUT);
  SERVO_Enable(0);
  SERVO_Enable(1);
  SERVO_SetAngle(0,0);
  SERVO_SetAngle(1,180);
}

void loop()
{
  char tache = '0';
  

    while (1)
    {

      switch (tache)
      {

      case '3': //déplacement

        for (int b=0;b<2;b++){
        for(int j=0;j<50;j++){SERVO_SetAngle(1,0+j);delay(10);}
        for(int j=50;j>0;j--){SERVO_SetAngle(1,0+j);delay(10);}
        }
        while (1)
        {
          tache='0';
          deplacement();
          if (findHuman == 1)
          {
            MOTOR_SetSpeed(0,-0.10);
            MOTOR_SetSpeed(1,-0.10);
            delay(1000);
            MOTOR_SetSpeed(0,0);
            MOTOR_SetSpeed(1,0);
            comMarketing('1');break;
          }
        }
        break;

       case '4': //danse1
        tache='0';
        danse();
        comMarketing('2');
        break;


      case '5': //danse2
        tache='0';
        danse2();
        comMarketing('2');
        break;

      case '6': //danse3
        tache='0';
        danse3();
        comMarketing('2');
        break;

        default:
        
        tache=comMarketing('0');
      }
    }
  
#ifdef DEBUGGING_CAPTEURS
  Serial.print("SRF0_1  ");
  Serial.print(SONAR_GetRange(0));

  // Serial.print(detect_couleur());
  // Serial.print("  Ligne:");

  getLigne(ligne);
  for (int i = 0; i < 8; i++)
  {
    Serial.print(ligne[i]);
  }
  Serial.println("");

#endif
}

//================================================================================================================================
//                                                              Fonctions
//================================================================================================================================

float *cartesien()
{

  static float position_robot[3];

  // update du temps écoulé
  myTime = millis();

  // on ajoute un délais avant d'actualiser la position du robot (augmente la stabilité du PID)
  if (myTime - elapsedTime > 10)
  {

    // Lecture des encodeurs et calcul des distances parcourues par les roues en cm
    Cart.dL = ENCODER_ReadReset(0) * CM_PER_TICKS;
    Cart.dR = ENCODER_ReadReset(1) * CM_PER_TICKS;

    // Calcul de la distance parcourue par le robot en cm
    Cart.dC = (Cart.dR + Cart.dL) / 2.0;

    Cart.Dtheta = (Cart.dR - Cart.dL) / (TRACK_WIDTH); // Calcul de la variation de la direction du robot en radians
    Cart.theta += Cart.Dtheta;                         // Direction du robot en radians

    // Orientation du robot [-PI, PI]
    if (Cart.theta > PI)
    {
      Cart.theta = Cart.theta - 2 * PI;
    }
    if (Cart.theta < -PI)
    {
      Cart.theta = Cart.theta + 2 * PI;
    }

    Cart.Dx = Cart.dC * cos(Cart.theta); // Distance parcourue en X pour ce cycle cm
    Cart.Dy = Cart.dC * sin(Cart.theta); // Distance parcourue en Y pour ce cycle cm
    Cart.coordx += Cart.Dx * 10;         // Coordonné en X du robot mm
    Cart.coordy += Cart.Dy * 10;         // Coordonné en Y du robot mm

    // section vitesses
    Cart.vL = Cart.dL / ((myTime - elapsedTime) / 1000.0); // vitesses en cm par secondes
    Cart.vR = Cart.dR / ((myTime - elapsedTime) / 1000.0);
    elapsedTime = myTime;

#ifdef DEBUGGING_CARTESIEN
    // Impression des données
    Serial.print("Coordx:");
    Serial.print(Cart.coordx);
    Serial.print(",");
    Serial.print("Coordy:");
    Serial.print(Cart.coordy);
    Serial.print(",");
    Serial.print("theta:");
    Serial.println(Cart.theta);
// Serial.print(",");
// Serial.print("  vL: ");
// Serial.print(Cart.vL);
// Serial.print(",");
// Serial.print("  vR: ");
// Serial.print(Cart.vR);
// Serial.print(",");
// Serial.print("  w: ");
// Serial.println(Cart.w);
#endif

    // Positionnement global du robot (State)
    position_robot[0] = Cart.coordx;
    position_robot[1] = Cart.coordy;
    position_robot[2] = Cart.theta;
    return position_robot;
  }
  return position_robot;
}

void pid(float position_robot[])
{
  // calcul de la distance entre le GOAL et le robot
  PID.Dx = (*(*(ptrPoint + point_i) + 0) - position_robot[0]);
  PID.Dy = (*(*(ptrPoint + point_i) + 1) - position_robot[1]);

#ifdef DEBUGGING_DISTANCE_ROB_POINT
  Serial.print("position_robotX");
  Serial.print(position_robot[0]);
  Serial.print("position_robotY");
  Serial.print(position_robot[1]);
  Serial.print("pointX");
  Serial.print(*(*(ptrPoint + point_i) + 0));
  Serial.print("pointY");
  Serial.print(*(*(ptrPoint + point_i) + 1));
  Serial.print("Dx");
  Serial.print(PID.Dx);
  Serial.print("Dy");
  Serial.println(PID.Dy);
#endif

  // calcul de l'angle voulu
  PID.SP1 = atan2f(PID.Dy, PID.Dx);

  // calcul de l'erreur entre l'angle voulu et l'angle du robot
  PID.eTheta_KP = PID.SP1 - position_robot[2];
  if (abs(PID.eTheta_KP) > PI)
  {
    PID.eTheta_KP = 2 * PI - abs(PID.eTheta_KP);
    if (position_robot[2] < 0 and position_robot[2] >= -PI and PID.SP1 > 0 and PID.SP1 <= PI)
    {
      PID.eTheta_KP = PID.eTheta_KP * -1;
    }
    if (position_robot[2] > 0 and position_robot[2] <= PI and PID.SP1 < 0 and PID.SP1 >= -PI)
    {
      // eTeta_KP= eTeta_KP *-1;
    }
  }

  // addition de tous les erreurs pour faire l'intégrale des erreurs
  PID.eTheta_KI += PID.eTheta_KP;

  // KI1 * eTeat_KI + on calcule la réponse de la boucle PID. ICI nous avons seulment une réponse qui travaile sur le gain proportionnel et sur l'intégralle du procédé
  PID.CV_w = KP1 * PID.eTheta_KP + KI1 * PID.eTheta_KI;

  PID.CV_vl = speed; // 0.2======================================================================================================================================================================================================================
  if (PID.eTheta_KP > PI or PID.eTheta_KP < -PI)
  {
    PID.CV_vl = 0;
  }

  PID.SP2 = (((TRACK_WIDTH * PID.CV_w) / RADIUS_WHEEL_CM) + Cart.vL);
  PID.eVr_KP = PID.SP2 - Cart.vR;
  PID.eVr_KI += PID.eVr_KP;

  PID.CV_vr = KP2 * PID.eVr_KP + KI2 * PID.eVr_KI;
  // if(PID.CV_vr>0.5){PID.CV_vr=0.5;}
  MOTOR_SetSpeed(1, PID.CV_vr);
  MOTOR_SetSpeed(0, PID.CV_vl);

#ifdef TUNNING_PID_Theta

  Serial.print(" eTeta_KP:");
  Serial.print(PID.eTheta_KP);
  Serial.print(",");
  Serial.print("  SP1:");
  Serial.print(PID.SP1);
  Serial.print(",");
  Serial.print("  theta:");
  Serial.println(position_robot[2]);

#endif

#ifdef TUNNING_PID_VR
  Serial.print("CV_vr:");
  Serial.print(PID.CV_vr);
  Serial.print(",");
  Serial.print("CV_vl:");
  Serial.print(PID.CV_vl);
  Serial.print(",");
  Serial.print("vr:");
  Serial.print(Cart.vR);
  Serial.print(",");
  Serial.print("w:");
  Serial.print(PID.CV_w);
  Serial.print(",");
  Serial.print("SP2:");
  Serial.println(PID.SP2);
#endif
}

void depart()
{
  bool orrientation_trouve = 0;

  int ligne_reached = 0;

  while (orrientation_trouve == 0)
  {

    for (int a = 0; a < 2; a++) // alignement sur une ligne noire (a) fois
    {
      MOTOR_SetSpeed(0, 0.15);
      MOTOR_SetSpeed(1, 0.15);
      ligne_reached = 0;
      bool ligne_reached_g = 0;
      bool ligne_reached_d = 0;
      while (ligne_reached < 2)
      {

        getLigne(ligne);
        if ((ligne[0] == 0) and ligne_reached_d == 0) // if (((ligne[0] == 0 or ligne[5] ==0)and ligne[7] == 1)  and ligne_reached != 1)
        {
          MOTOR_SetSpeed(1, -0.10);
          ligne_reached_d = 1;
        }

        if ((ligne[7] == 0) and ligne_reached_g == 0) // if (((ligne[0] == 1 or ligne[5] ==0) and ligne[7] == 0) and ligne_reached != 1)
        {
          MOTOR_SetSpeed(0, -0.10);
          ligne_reached_g = 1;
        }
        if ((ligne[0] == 0 and ligne[7] == 0) or (ligne_reached_d and ligne_reached_g)) // if ((ligne[0] == 0 or ligne[5] ==0)and ligne[7] == 0)
        {
          MOTOR_SetSpeed(0, -0.10);
          MOTOR_SetSpeed(1, -0.10);
          ligne_reached = 2;
        }
      }
      delay(1000);
      // MOTOR_SetSpeed(0,0);
      // MOTOR_SetSpeed(1,0);
    }
    orrientation_trouve = 1;
  }

  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);

  getLigne(ligne);

  MOTOR_SetSpeed(0, 0.15);
  MOTOR_SetSpeed(1, 0.15);

  while (ligne[3] && ligne[4]) // arrête à la prochaine ligne noir perpendiculaire
  {
    getLigne(ligne);
  }

  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);

  delay(100);

  ENCODER_Reset(0);
  ENCODER_Reset(1);

  while (1)
  {

    ptrPoint = Megagenial;
    break;
  }
  findHuman = 0;
  speed = 0.2;
  rCompletion_decroisant = 0;
  Cart.theta = PI / 2;          // angle_depart;
  Cart.coordx = ptrPoint[0][0]; // 110 = distance de la face du capteur au centre du robot // 1221.401 = coordoné en x du mur de depart
  Cart.coordy = ptrPoint[0][1];
  SERVO_SetAngle(0,0);
  SERVO_SetAngle(1,180);
}

void deplacement()
{

#ifndef LOOP_DE_TEST

  depart();

  myMouvmentTime = millis();
  myMouvmentDelay = myMouvmentTime;
  for (point_i = 0; point_i < 2;)
  {
    myMouvmentTime = millis();
    

    if (SONAR_GetRange(0) < 15.0 and findHuman == 0 and myMouvmentTime - myMouvmentDelay > 3000)
    {
      speed = 0;
      Cart.theta += -PI / 4;
      findHuman = 1;
      myMouvmentDelay = myMouvmentTime;
      rCompletion_decroisant = 1000;
    }
    if (myMouvmentTime - myMouvmentDelay > 5000 and findHuman == 1)
    {
      for (int b=0;b<2;b++){
      for(int j=0;j<50;j++){SERVO_SetAngle(1,0+j);delay(10);}
      for(int j=50;j>0;j--){SERVO_SetAngle(1,0+j);delay(10);}
      }
      SERVO_SetAngle(1,180);
      Cart.theta=PI/4;
      break;
    }
    getLigne(ligne);
    if ((not ligne[3] && not ligne[4] && not findHuman) and myMouvmentTime - myMouvmentDelay > 1000) // arrête à la prochaine ligne noir perpendiculaire
    {
      break;
    }

    float *posRobot = cartesien();

// Exécute la fonction "pid" avec la position à atteindre
#ifndef PAS_DE_MOUVEMENT
    pid(posRobot);
#endif

    // Si le robot se trouve dans la zone de complétion du point, passer au prochain point
    if (hypot(*(*(ptrPoint + point_i) + 0) - posRobot[0], *(*(ptrPoint + point_i) + 1) - posRobot[1]) < rCompletion + rCompletion_decroisant)
    { // 4cm de rayon pour atteindre un point
      point_i++;
      if (rCompletion_decroisant > 0)
      {
        rCompletion_decroisant -= 100;
      }
      else
      {
        rCompletion_decroisant = 0;
      }
      // Serial.println("Arrive");
      // cycle=0;
      // eVr_KI=0;
      //  eTheta_KI=0;
    }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);

#endif
}

void getLigne(bool dLigne[8])
{
  dLigne[0] = !digitalRead(LIGNE_1);
  dLigne[1] = !digitalRead(LIGNE_2);
  dLigne[2] = !digitalRead(LIGNE_3);
  dLigne[3] = !digitalRead(LIGNE_4);
  dLigne[4] = !digitalRead(LIGNE_5);
  dLigne[5] = !digitalRead(LIGNE_6);
  dLigne[6] = !digitalRead(LIGNE_7);
  dLigne[7] = !digitalRead(LIGNE_8);
}

void danse()
{
   
  bool orientationclose = 0;
  MOTOR_SetSpeed(0, -0.3);
  MOTOR_SetSpeed(1, -0.3);
  delay(1000);
  myMouvmentTime = millis();
  myMouvmentDelay = myMouvmentTime;
  MOTOR_SetSpeed(0, 0.3);
  MOTOR_SetSpeed(1, -0.3);
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  while (myMouvmentTime - myMouvmentDelay < 4000 or not orientationclose)
  {
    myMouvmentTime = millis();

    cartesien();
    SERVO_SetAngle(0,180*sin(abs(Cart.theta)/2));
    SERVO_SetAngle(1,180*cos(abs(Cart.theta)/2));
    if (Cart.theta > (PI / 2) - (PI / 36) and Cart.theta < (PI / 2) + (PI / 36))
    {
      orientationclose = 1;
    }
    else
    {
      orientationclose = 0;
    } // Cart.theta>PI/4 and Cart.theta<3*PI/4){orientationclose=1;}else{orientationclose=0;}
  }

  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  depart();
}

void danse2(){
 

  MOTOR_SetSpeed(0, -0.3);
  MOTOR_SetSpeed(1, -0.3);
  delay(1000);


  ENCODER_Reset(0);
  ENCODER_Reset(1);

  while (Cart.theta>0)
  {
   
    MOTOR_SetSpeed(0,0.3);
    MOTOR_SetSpeed(1,-0.3);
    cartesien();
    SERVO_SetAngle(0,180*sin(abs(Cart.theta)/2));
    SERVO_SetAngle(1,180*cos(abs(Cart.theta)/2));
  }

  while (Cart.theta>-PI/4 and Cart.theta<PI)
  {
   
    MOTOR_SetSpeed(0,-0.3);
    MOTOR_SetSpeed(1,0.3);
    cartesien();
    SERVO_SetAngle(0,180*sin(abs(Cart.theta)/2));
    SERVO_SetAngle(1,180*cos(abs(Cart.theta)/2));
  }

  while (Cart.theta<-3*PI/4 or Cart.theta>PI/2)
  {
   
    MOTOR_SetSpeed(0,0.3);
    MOTOR_SetSpeed(1,-0.3);
    cartesien();
    SERVO_SetAngle(0,180*sin(abs(Cart.theta)/2));
    SERVO_SetAngle(1,180*cos(abs(Cart.theta)/2));
  }

  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  depart();

}

void danse3(){

  
  MOTOR_SetSpeed(0, -0.3);
  MOTOR_SetSpeed(1, -0.3);
  delay(2000);
 

  ENCODER_Reset(0);
  ENCODER_Reset(1);

  while (Cart.theta>0)
  {
   
    MOTOR_SetSpeed(0,0.3);
    MOTOR_SetSpeed(1,0);
    cartesien();
    SERVO_SetAngle(0,180*sin(abs(Cart.theta)/2));
    SERVO_SetAngle(1,180*cos(abs(Cart.theta)/2));
  }

  while (Cart.theta>-PI/4 and Cart.theta<PI)
  {
   
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(1,0.3);
    cartesien();
    SERVO_SetAngle(0,180*sin(abs(Cart.theta)/2));
    SERVO_SetAngle(1,180*cos(abs(Cart.theta)/2));
  }

  while (Cart.theta<-3*PI/4 or Cart.theta>PI/2)
  {
   
    MOTOR_SetSpeed(0,0.3);
    MOTOR_SetSpeed(1,0);
    cartesien();
    SERVO_SetAngle(0,180*sin(abs(Cart.theta)/2));
    SERVO_SetAngle(1,180*cos(abs(Cart.theta)/2));
  }

  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  depart();

}