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
#define TEST_BLEU
//#define DEUXIEME_TOUR
//#define ETAT_BRAS
//#define TEST_SANS_BRAS
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

float Spirale[][2]={{0.0,-0.0},{-32.18331358525289,78.27947394830758},{-52.17526813848373,160.49218536080897},{-60.026221494162776,244.81673508777698},{-55.78653148676028,329.43172397948445},{-39.50655595074651,412.5157528862041},{-11.23665272059176,492.2474226582087},{28.94570230688324,566.7763562465426},{80.16534309188258,634.1116813421103},{140.85890137725735,693.0935669757914},{209.4741779614504,742.767702393527},{284.45897364290414,782.1797768412582},{364.26108922006154,810.3754795649265},{447.3283254913654,826.4004998104731},{531.9468972171496,829.4728488492366},{616.0656288146997,819.82958068483},{697.783528257615,797.76884921994},{775.1880603110469,763.553673341131},{846.3458972922522,717.6838132584058},{909.5445745029905,661.3575939157948},{963.1714999449656,595.9000187966694},{1005.6200027109777,522.6755575871723},{1035.6777688945053,443.5331866184492},{1052.6295464308762,360.6146554260427},{1055.787025083775,276.06245194446836},{1044.6737096131044,192.1490830355574},{1019.7238697126195,111.2626046854152},{981.6116401941963,35.72682329289385},{931.0617337548105,-32.126414370738445},{869.5458734856918,-90.23960165149263},{799.0546805693186,-137.0763564963957},{721.5914126391673,-171.1441230227526},{639.4147397541666,-191.34083396571148},{554.9933523342872,-197.34870031821015},{470.7856936595899,-188.94270411979969},{389.2824525288888,-166.1819286871019},{312.77817796415894,-129.98470597776142},{243.46493783842186,-81.43022733730352},{183.4819262555486,-21.737803807095727},{134.41851825428716,47.22391204525333},{97.57581602641365,123.41474474382107},{74.3327223026461,204.9132648336102},{66.10414567465693,289.4195695474819},{73.80553887227362,373.63570617787104},{98.3133648853013,454.2193205265763},{140.23656590125032,527.6056943165034},{198.07009488105356,589.2362392263218},{269.3312318588124,634.6604170946028},{349.9010602210644,660.0241574884565},{434.24216917816864,662.3505195607477},{516.1013725256546,641.5967707208756},{589.9498074335241,600.3654497254046},{650.8407654074405,541.8683228092668},{696.2203002501692,470.5051754480156},{724.2346568553809,390.6499354885887},{733.2104202023527,306.38599367398984},{722.2436412859669,222.6231778956697},{690.5774825339757,144.5033351483877},{638.8266384260912,77.7389065948313},{570.3375113758226,28.388571485857668},{490.29188965458957,1.6803691432795251},{405.92531053188463,1.3327726861799982},{325.744713820514,27.557964960261714},{258.15670790698243,78.05302809598363},{210.18362746223178,147.42549394334972},{187.15264189998868,228.58374917902907},{191.55557443911766,312.80082084270447},{223.09440565011525,391.0360659461832},{278.5915160417563,454.54246258844205},{352.8021560213972,494.34096742047194},{436.53804104321773,502.11584835855615},{515.5104745419757,473.47194073524855},{572.1936713097934,411.65226198927996},{590.7706275001742,330.1167656439043},{566.480180864213,249.80577061861734},{507.553811896467,189.99363966298577},{428.2842979585859,162.5689131924592},{351.42400656091917,191.10297763190223},{341.9308668205066,269.32342770768395},{417.16421149544004,296.4457754583175},{497.1645307456394,269.06380957490467},{576.9086251990508,240.3371096101764},{656.9062974573992,212.32293236799546},{735.5207157974053,180.93025593961335},{811.1360273354418,141.81846902196853},{880.0735473923318,92.21313183189088},{937.3152736570496,30.78429373523408},{977.5320112964502,-43.28303221027134},{996.0872633626266,-125.69487613809707},{992.2934888678699,-210.1143947626354},{965.8309540758129,-290.29782545043213},{918.0426668905516,-359.90800087668464},{853.4036358278105,-414.3687689130737},{776.7079851860863,-449.6889308971518},{693.2074242233062,-462.22861434386226},{609.371937394344,-451.64962651121857},{531.2929051392258,-419.3912333768553},{465.07027145644474,-367.0100819745866},{415.9353482708024,-298.3207648052839},{386.6549735234582,-219.03689743431525},{379.69097,-134.89021}};


float Cercle[][2]={{0.0,-0.0},{-16.38947664449221,30.606147976650426},{-30.806899428356004,62.18709421154414},{-43.21702721666757,94.61131688379727},{-53.584618874503086,127.74729417252594},{-61.87443326693878,161.4635042568463},{-68.0512292590508,195.6284253158744},{-72.07976571591536,230.1105355287265},{-73.92480150260864,264.77831307451856},{-73.55457605300603,299.4956562459527},{-70.97779078105437,334.11615828362653},{-66.22925177860884,368.50715233224116},{-59.344200223335754,402.5371165694758},{-50.35787729290143,436.07452917300975},{-39.30552416497217,468.9878683205224},{-26.222382017214333,501.1456121896929},{-11.143692027294145,532.4162389582007},{5.8943554260013356,562.6674528992339},{24.82609257586922,591.7681115008871},{45.555962897326886,619.6162270533417},{67.9876858555717,646.115518772078},{92.024980915801,671.1697058725757},{117.57156754321224,694.6825075703144},{144.53116520300284,716.5576430807744},{172.80749336037007,736.6988316194354},{202.30427148051083,755.0097924017791},{232.9104198572883,771.3992690310403},{264.49136637832595,785.8166919635306},{296.9155892270902,798.2268199514431},{330.05156658704755,808.5944117469726},{363.76777664166485,816.8842261023128},{397.93269757440845,823.0610217696571},{432.41480756874495,827.0895575011998},{467.08258480814095,828.9345920491347},{501.79992742137455,828.564364993105},{536.4204293275407,825.9875785109721},{570.8114236986896,821.2390387230892},{604.8413885153187,814.3539867724064},{638.378801757925,805.3676638018727},{671.2921414070058,794.3153109544385},{703.4498854430586,781.2321693730526},{734.7205118465804,766.1534802006652},{764.9717246673329,749.115432673588},{794.0723818824567,730.1836937263132},{821.9204962351698,709.4538221915125},{848.4197870542596,687.0220985330393},{873.4739736685128,662.9848032147465},{896.9867754067159,637.4382167004877},{918.8619115976558,610.4786194541155},{939.0031015701194,582.2022919394834},{957.3140646528896,552.7055146204431},{973.7035388591767,522.0993648852311},{988.1209636585297,490.51841789775455},{1000.5310966693283,458.0941953459907},{1010.8986955099525,424.9582189179167},{1019.1885177987818,391.2420103015099},{1025.3653211541957,357.077091184747},{1029.393863194574,322.59498325560594},{1031.2389015382962,287.92720820206307},{1030.868674533153,253.20986527011806},{1028.291884051494,218.58936312551506},{1023.5433374868252,184.19836974957227},{1016.6582773256353,150.16840674139104},{1007.6719460544143,116.63099570007306},{996.6195861596509,83.71765822471973},{983.5364401278341,51.559915914432615},{968.457750445453,20.28929036831318},{951.4197059024673,-9.961923065363065},{932.4879683239388,-39.06258160473669},{911.7580980892245,-66.91069709136069},{889.3263756620037,-93.40998891640362},{865.2890815059573,-118.46417647103304},{839.7424960847646,-141.97697914641685},{812.7828998621062,-163.85211633372305},{784.506573301662,-183.99330742411945},{755.0097968671117,-202.3042718087736},{724.4036465194635,-218.69374615804816},{692.8226987415045,-233.11116766649602},{660.3984752387382,-245.52129498460056},{627.2624977166688,-255.88888676284498},{593.5462878807995,-264.17870165171246},{559.3813674366343,-270.3554983016863},{524.899258089677,-274.3840353632496},{490.23148154543094,-276.22907148688586},{455.51413947779815,-275.8588459507322},{420.8936384877963,-273.28206032432905},{386.50264527267257,-268.53352087893967},{352.47268153793084,-261.6484689640807},{318.93526898907464,-252.6621459292688},{286.0219293316076,-241.60979312402085},{253.8641842710337,-228.52665189785353},{222.59355551285654,-213.4479636002837},{192.34233996109046,-196.40991754832905},{163.24168220295596,-177.4781796493336},{135.39356763025535,-156.74830859214467},{108.89427683069961,-134.3165849852092},{83.84009039200048,-110.27928943697496},{60.327288901869466,-84.73270255588943},{38.452152948018075,-57.77310495040013},{18.31096311815775,-29.496777228954485},{0.0,-0.0}};

float Triangle[][2]={{0.0,-0.0},{14.359710473209566,24.871749000007778},{28.71942094641913,49.743498000015556},{43.079131419628695,74.61524700002333},{57.43884189283826,99.48699600003111},{71.79855236604783,124.35874500003891},{86.15826283925739,149.23049400004666},{100.51797331246696,174.10224300005447},{114.87768378567652,198.97399200006222},{129.2373942588861,223.84574100007003},{143.59710473209566,248.71749000007782},{157.95681520530525,273.58923900008557},{172.31652567851478,298.4609880000933},{186.67623615172437,323.3327370001012},{201.03594662493393,348.20448600010894},{215.3956570981435,373.0762350001167},{229.75536757135305,397.94798400012445},{244.11507804456264,422.8197330001323},{258.4747885177722,447.69148200014007},{272.83449899098173,472.56323100014777},{287.1942094641913,497.43498000015563},{301.55391993740085,522.3067290001634},{315.9136304106105,547.1784780001711},{330.27334088382,572.0502270001789},{344.63305135702956,596.9219760001866},{358.99276183023915,621.7937250001945},{373.35247230344874,646.6654740002024},{387.71218277665827,671.53722300021},{402.07189324986786,696.4089720002179},{416.4316037230774,721.2807210002255},{430.791314196287,746.1524700002334},{445.1510246694965,771.0242190002411},{459.5107351427061,795.8959680002489},{473.8704456159157,820.7677170002568},{488.23015668912564,812.477114346146},{502.58986806233514,787.6053658657535},{516.9495794355447,762.733617385361},{531.3092908087542,737.8618689049684},{545.6690021819638,712.9901204245758},{560.0287135551735,688.1183719441833},{574.388424928383,663.2466234637908},{588.7481363015925,638.3748749833984},{603.1078476748021,613.5031265030058},{617.4675590480116,588.6313780226133},{631.8272704212212,563.7596295422206},{646.1869817944307,538.8878810618282},{660.5466931676403,514.0161325814356},{674.9064045408498,489.14438410104316},{689.2661159140594,464.27263562065065},{703.625827287269,439.4008871402581},{717.9855386604786,414.5291386598655},{732.3452500336881,389.657390179473},{746.7049614068976,364.7856416990805},{761.0646727801072,339.91389321868786},{775.4243841533169,315.0421447382953},{789.7840955265265,290.1703962579028},{804.143806899736,265.29864777751027},{818.5035182729455,240.42689929711798},{832.863229646155,215.55515081672547},{847.2229410193645,190.68340233633285},{861.5826523925741,165.81165385594034},{875.9423637657837,140.9399053755477},{890.3020751389932,116.0681568951552},{904.6617865122028,91.19640841476269},{919.0214978854124,66.32465993437006},{933.381209258622,41.45291145397755},{947.7409206318316,16.581162973585037},{947.7409163363371,-0.0},{919.0214946291753,-0.0},{890.3020729220138,-0.0},{861.5826512148521,-0.0},{832.8632295076903,-0.0},{804.1438078005286,-0.0},{775.4243860933668,-0.0},{746.7049643862051,-0.0},{717.9855426790433,-0.0},{689.2661209718816,-0.0},{660.5466992647198,-0.0},{631.8272775575581,-0.0},{603.1078558503963,-0.0},{574.3884341432345,-0.0},{545.6690124360728,-0.0},{516.9495907289113,-0.0},{488.23016902174953,-0.0},{459.5107473145879,-0.0},{430.7913256074261,-0.0},{402.07190390026426,-0.0},{373.35248219310256,-0.0},{344.63306048594075,-0.0},{315.91363877877905,-0.0},{287.19421707161723,-0.0},{258.47479536445553,-0.0},{229.75537365729372,-0.0},{201.03595195013202,-0.0},{172.31653024297054,-0.0},{143.59710853580884,-0.0},{114.87768682864703,-0.0},{86.15826512148521,-0.0},{57.438843414323514,-0.0},{28.7194217071617,-0.0},{0.0,-0.0}};

float Megagenial[][2]={{0.0,-0.0},{20.0,-0.0},{40.0,-0.0},{60.0,-0.0},{80.0,-0.0},{100.0,-0.0},{120.0,-0.0},{140.0,-0.0},{160.0,-0.0},{180.0,-0.0},{200.0,-0.0},{220.0,-0.0},{240.0,-0.0},{260.0,-0.0},{280.0,-0.0},{300.0,-0.0},{320.0,-0.0},{340.0,-0.0},{360.0,-0.0},{380.0,-0.0},{400.0,-0.0},{420.0,-0.0},{440.0,-0.0},{460.0,-0.0},{480.0,-0.0},{500.0,-0.0},{520.0,-0.0},{540.0,-0.0},{560.0,-0.0},{580.0,-0.0},{600.0,-0.0},{620.0,-0.0},{640.0,-0.0},{660.0,-0.0},{680.0,-0.0},{700.0,-0.0},{720.0,-0.0},{740.0,-0.0},{760.0,-0.0},{780.0,-0.0},{800.0,-0.0},{820.0,-0.0},{840.0,-0.0},{860.0,-0.0},{880.0,-0.0},{900.0,-0.0},{920.0,-0.0},{940.0,-0.0},{960.0,-0.0},{980.0,-0.0},{1000.0,-0.0},{1020.0,-0.0},{1040.0,-0.0},{1060.0,-0.0},{1080.0,-0.0},{1100.0,-0.0},{1120.0,-0.0},{1140.0,-0.0},{1160.0,-0.0},{1180.0,-0.0},{1200.0,-0.0},{1220.0,-0.0},{1240.0,-0.0},{1260.0,-0.0},{1280.0,-0.0},{1300.0,-0.0},{1320.0,-0.0},{1340.0,-0.0},{1360.0,-0.0},{1380.0,-0.0},{1400.0,-0.0},{1420.0,-0.0},{1440.0,-0.0},{1460.0,-0.0},{1480.0,-0.0},{1500.0,-0.0},{1520.0,-0.0},{1540.0,-0.0},{1560.0,-0.0},{1580.0,-0.0},{1600.0,-0.0},{1620.0,-0.0},{1640.0,-0.0},{1660.0,-0.0},{1680.0,-0.0},{1700.0,-0.0},{1720.0,-0.0},{1740.0,-0.0},{1760.0,-0.0},{1780.0,-0.0},{1800.0,-0.0},{1820.0,-0.0},{1840.0,-0.0},{1860.0,-0.0},{1880.0,-0.0},{1900.0,-0.0},{1920.0,-0.0},{1940.0,-0.0},{1960.0,-0.0},{1980.0,-0.0},{2000.0,-0.0}};


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



float speed=0.2;

//================================================================================================================================
//                                                     Déclaration des fonctions
//================================================================================================================================

float *cartesien(); // Positionnement global (coordonnées et orientation)
void pid(float *);  // PID 1 moteur avec erreur sur l'orientation


void depart();

void getLigne(bool dLigne[8]);


//================================================================================================================================
//                                                         Programmes Arduino
//================================================================================================================================

void setup()
{
  BoardInit();          // Initialisation de librobus
  Serial.begin(115200); // Modification du Baud Rate
  pinMode(LIGNE_1, INPUT);
  pinMode(LIGNE_2, INPUT);
  pinMode(LIGNE_3, INPUT);
  pinMode(LIGNE_4, INPUT);
  pinMode(LIGNE_5, INPUT);
  pinMode(LIGNE_6, INPUT);
  pinMode(LIGNE_7, INPUT);
  pinMode(LIGNE_8, INPUT);

}

void loop()
{

#ifndef LOOP_DE_TEST
int rCompletion_decroisant=400;//750 limite pour bleu
if(ROBUS_IsBumper(3)){

 while(1){
  

 
 

   
    depart();

    for (point_i = 0; point_i < 101;)
    {

      float *posRobot = cartesien();

// Exécute la fonction "pid" avec la position à atteindre
#ifndef PAS_DE_MOUVEMENT
      pid(posRobot);
#endif
      
      // Si le robot se trouve dans la zone de complétion du point, passer au prochain point
      if (hypot(*(*(ptrPoint + point_i) + 0) - posRobot[0], *(*(ptrPoint + point_i) + 1) - posRobot[1]) < rCompletion)//+rCompletion_decroisant)
      { // 4cm de rayon pour atteindre un point
        point_i++;
        if(rCompletion_decroisant>0){rCompletion_decroisant-=100;}
        else{rCompletion_decroisant=0;}
        // Serial.println("Arrive");
        // cycle=0;
        // eVr_KI=0;
        //  eTheta_KI=0;
      }

#ifdef DEBUGGING_CAPTEURS
      Serial.print("IR0:  ");
      Serial.print(getDistance(0));
      Serial.print("  IR2:  ");
      Serial.print(getDistance(2));
      Serial.print("  IR3:  ");
      Serial.print(getDistance(3));
      Serial.print("  Couleur: ");
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
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(1,0);


   
  }
#endif
#ifdef DEBUGGING_CAPTEURS
  Serial.print("IR0:  ");
  Serial.print(getDistance(0));
  Serial.print("  IR2:  ");
  Serial.print(getDistance(2));
  Serial.print("  IR3:  ");
  Serial.print(getDistance(3));
  Serial.print("  Couleur: ");
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

  

  PID.CV_vl = speed;//0.2======================================================================================================================================================================================================================
  if (PID.eTheta_KP > PI or PID.eTheta_KP < -PI)
  {
    PID.CV_vl = 0;
  }

  PID.SP2 = (((TRACK_WIDTH * PID.CV_w) / RADIUS_WHEEL_CM) + Cart.vL);
  PID.eVr_KP = PID.SP2 - Cart.vR;
  PID.eVr_KI += PID.eVr_KP;

  PID.CV_vr = KP2 * PID.eVr_KP + KI2 * PID.eVr_KI;
  //if(PID.CV_vr>0.5){PID.CV_vr=0.5;}
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

    for (int a = 0; a < 3; a++) //alignement sur une ligne noire (a) fois
    {
      MOTOR_SetSpeed(0, 0.15);
      MOTOR_SetSpeed(1, 0.15);
      ligne_reached = 0;
      while (ligne_reached < 2)
      {

        getLigne(ligne);
        if (ligne[0] == 0 and ligne[7] == 1 and ligne_reached != 1)
        {
          MOTOR_SetSpeed(1, -0.10);
          ligne_reached = 1;
        }

        if (ligne[0] == 1 and ligne[7] == 0 and ligne_reached != 1)
        {
          MOTOR_SetSpeed(0, -0.10);
          ligne_reached = 1;
        }
        if (ligne[0] == 0 and ligne[7] == 0)
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

  while (ligne[3] && ligne[4])//arrête à la prochaine ligne noir perpendiculaire
  {
    getLigne(ligne);
  }

  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);


  delay(100);


  ENCODER_Reset(0);
  ENCODER_Reset(1);


while(1){

  if(ROBUS_IsBumper(0)){ptrPoint=Cercle;break;}
  if(ROBUS_IsBumper(1)){ptrPoint=Triangle;break;}
  if(ROBUS_IsBumper(2)){ptrPoint=Spirale;break;}
  if(ROBUS_IsBumper(3)){ptrPoint=Megagenial;break;}
  

}

  Cart.theta = PI / 2; // angle_depart;
  Cart.coordx = ptrPoint[0][0]; //110 = distance de la face du capteur au centre du robot // 1221.401 = coordoné en x du mur de depart
  Cart.coordy = ptrPoint[0][1];

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



