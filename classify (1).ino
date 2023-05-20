#define MIC_INPUT                   A2
#define RXLED                       17
#define TXLED                       30

#define SIZE                        5504
#define ADC_TIMER_MS                0.35
#define AVG_SHIFT                   5
#define AVG_SIZE                    (int) pow(2, AVG_SHIFT)
#define SIZE_AFTER_FILTER           (int) SIZE / AVG_SIZE

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

#define SNIPPET_SIZE                100
#define PRELENGTH                   5
#define THRESHOLD                   0.5
#define BASIS_DIM                   3

#define EUCLIDEAN_THRESHOLD         0.027
#define LOUDNESS_THRESHOLD          50

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

String words[4] = {"tasmanian devil", "dog", "salamander", "dragon"};

float pca_vec1[100] = {-0.005690597910820786, 0.0004337795455885418, -0.0008836718843184821, 0.006392739283016022, 0.013298735411946379, 0.08542943688299914, 0.10276657255031124, 0.11309401764827703, 0.11681835229016314, 0.12379615068715973, 0.12176505052496854, 0.1160101466695292, 0.1380880895484472, 0.1656367321451668, 0.20465290268215217, 0.22793141991000207, 0.23636811848577932, 0.23934087743998755, 0.2478401525485763, 0.23447116421996633, 0.23266337769489512, 0.2220226257323611, 0.21323783154097403, 0.19379744346602729, 0.18630616273316766, 0.1479835711423203, 0.09761013598471298, 0.053118943557107924, 0.022423630426670355, -0.02264318180227574, -0.04161932468057921, -0.05914595789226712, -0.06813611874157123, -0.061137247617187675, -0.0394749016483113, -0.026153182339095884, -0.01565313530503356, -0.014684961624194859, -0.019390479523479998, -0.033864282026071627, -0.04842518090919173, -0.05155392874573742, -0.060884170243009704, -0.06316470084333158, -0.06710973146730767, -0.07731411518516755, -0.07800872412707117, -0.08105235685470899, -0.08497050581208601, -0.0719653986414982, -0.0670705460486852, -0.056497966223052316, -0.04701193451020532, -0.04471440560173918, -0.04518942966835085, -0.0383250633935225, -0.037956408275049064, -0.037521295573767155, -0.037284363606547834, -0.035870307223764004, -0.04339411803379169, -0.051212193268009625, -0.0762263950451188, -0.08682893035621655, -0.10652382493787249, -0.11311867319288539, -0.12681679924261974, -0.1339479914001439, -0.13312694527357072, -0.13190587188385422, -0.12458072224628623, -0.10759438359989566, -0.09503638865224594, -0.07899232146072918, -0.06396108616773, -0.060381889276457365, -0.05815090600589126, -0.05344215192841561, -0.05099214576984963, -0.05603700798561359, -0.06175274805949222, -0.05935561142170482, -0.05599349944415075, -0.05592534845592056, -0.05439376807774869, -0.04844345729504309, -0.04184608037509619, -0.03754454641083311, -0.03325953638192741, -0.028165810271731314, -0.023257365593956015, -0.01625161612523492, -0.014813595540743916, -0.010478155151293796, -0.00851819789623239, -0.00803394671865223, -0.004870030687757358, -0.004086910024567455, -0.002070737689081906, -0.0015988774549068115};
float pca_vec2[100] = {0.06485568556334577, 0.09642390375595977, 0.1572939332436396, 0.21866841881617677, 0.15878678343623237, -0.10697075711907192, -0.16044586600125268, -0.1469898290247124, -0.04304784421043361, 0.001789757894018821, -0.03628796094683505, -0.014451903212901813, -0.0014989112755765734, 0.014350215004884797, -0.0021995063565736802, 0.01791664169340386, 0.011133315653726533, 0.02812617112150301, 0.003203029526544148, 0.019246510710519485, 0.017975797735148687, 0.049248743574451415, 0.05381935420360526, 0.06272806901675099, 0.02285123394760007, -0.05733180709879277, -0.19782255676068086, -0.2717710834189478, -0.33242730696225725, -0.3052046728393571, -0.2543303248611228, -0.16916534182094928, -0.06799464537692164, 0.028509674357743413, 0.14416629648010848, 0.21144666247288213, 0.2381607897941216, 0.2532303104354848, 0.24283047599231233, 0.2050555640077801, 0.16037826049003576, 0.14257401041588255, 0.1216404942066013, 0.0963336837887414, 0.06693614591778287, 0.05302086642399652, 0.03026918456680241, 0.019618880971522645, 0.0022611705669854654, -0.007779367125817937, -0.0040386892304921694, -0.0032283534156436868, -0.006112574568798284, -0.008914869416822681, -0.0001391278919917608, -0.014555390054676248, -0.011686626526115896, -0.009897635853375187, -0.009175041634397285, -0.010568196920480662, -0.017528927140373626, -0.019154275817811244, -0.015884900664982767, -0.029349515518223266, -0.0355780511207773, -0.0388400630173761, -0.05159634962062085, -0.056020114290618074, -0.057575345015742505, -0.0502222898563448, -0.04707406040507144, -0.046588181362426956, -0.046057017915222626, -0.029523204320900606, -0.027101059115003656, -0.01686619622921473, -0.013846028364779515, -0.024065119515739704, -0.018734288540678653, -0.020490920087167012, -0.020289309163767262, -0.02067886316223704, -0.019679428315404514, -0.018043689388635993, -0.01509441341842194, -0.02101797346322138, -0.01386371805801392, -0.008256198533972161, -0.0026851537694263534, -0.008100313392683403, 0.002222428714346764, 0.0007344826132192808, 0.005802152774122238, 0.005640553058874703, 0.008566354958772565, 0.007192275588843537, 0.009758963759768528, 0.001584082028009779, 0.0021402953351795646, 0.005349533892426339};
float pca_vec3[100] = {-0.01345255551062644, -0.03019851209884003, -0.06135981916472445, -0.11138002243602482, -0.11305598002928341, -0.12066187297498676, -0.16260447468119021, -0.19548787468152692, -0.204819225118763, -0.20720100152754914, -0.1963749025660061, -0.18609799876050573, -0.14445574082963109, -0.05744926704416821, -0.0015993159888234639, 0.051346136266824306, 0.09055532139747814, 0.09329842566157391, 0.0853402236145296, 0.09876980681604372, 0.07559027238916861, 0.06227463857669138, 0.013131095720845648, 0.01678084626276016, -0.037760086487196844, -0.0646729211764289, -0.046018723170594834, -0.02033256135873334, 0.06055247545825558, 0.1146111459174628, 0.1227633940137217, 0.1562447245962455, 0.14155098923819404, 0.15934907017428296, 0.14440830685978198, 0.13596951961636772, 0.10118059336329797, 0.03562420888369482, -0.0015414099616374258, -0.059401837205170976, -0.0838214230302881, -0.08942913127700057, -0.07602794795848773, -0.08975228742625134, -0.0690427044947235, -0.07953668369834224, -0.033504563936992096, -0.011650802578814162, 0.019242520535638623, 0.046837504306997776, 0.07053130895764272, 0.08482857634423195, 0.07982279721849438, 0.0781248464667059, 0.08700219386204211, 0.08882534948003186, 0.09010390055259447, 0.10182976266811561, 0.09225343986506361, 0.07834379378226258, 0.021081386244729414, -0.02960126455156319, -0.09502055222064988, -0.13663642775345766, -0.17219750491958283, -0.20137976785448228, -0.22361954566530262, -0.22031681222733981, -0.20786953104785968, -0.16126093807776912, -0.10603741330155149, -0.0239321076030532, 0.027263988860756053, 0.08259655091634238, 0.10386497737726269, 0.13302986716078966, 0.12824561109195967, 0.09618403148732134, 0.071369367701758, 0.0561011897381696, 0.0495534281730187, 0.048107625066384004, 0.03821380725965165, 0.04449313708343249, 0.046265733703821914, 0.047820148665586586, 0.052164689933567474, 0.04280803329645454, 0.04820531741445234, 0.04269270332367687, 0.05427916415718823, 0.0398688870035077, 0.041524833336599974, 0.03666092480903191, 0.033181599419258904, 0.03125886993388887, 0.01927616366386994, 0.016789435147207734, 0.01096970047834212, 0.00560515108085119};
float projected_mean_vec[3] = {0.06795458596941857, -0.017034111673533473, -0.022902227665210093};
float centroid1[3] = {-0.0680079280654256, -0.00216188452568558, -0.015135146674708454};
float centroid2[3] = {0.05717415575171157, -0.02221331465771976, -0.005267573156271924};
float centroid3[3] = {-0.020676948706772415, -0.005729340081644682, 0.023316724952370083};
float centroid4[3] = {0.03151072102048651, 0.030104539265050044, -0.002914005121389709};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

//data array and index pointer
int16_t out[SIZE_AFTER_FILTER] = {0};
volatile int re_pointer = 0;

int16_t re0[AVG_SIZE] = {0};
int16_t re1[AVG_SIZE] = {0};
int write_arr = 0;

// this function allows us to get the correct array to save data to
// we are using a concept known as "pointer juggling" to save memory
int16_t * get_re(int loc){
  switch(loc){
    case 0:
      return re0;
    case 1:
      return re1;
    default:
      return re0;
  }
}

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  pinMode(MIC_INPUT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  delay(1000);

  re_pointer = 0;
      
  cli();
 
  //set timer1 interrupt at 1Hz * SAMPLING INTERVAL / 1000
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15.624 * ADC_TIMER_MS;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();

  Serial.begin(38400);
  delay(1000);
}


void loop(void) {
  if (re_pointer%AVG_SIZE == 0 && re_pointer <= SIZE){
    write_arr = !write_arr;
    envelope(get_re(!write_arr), out, re_pointer>>AVG_SHIFT);
  }
  if (re_pointer == (int) (SIZE / 3)) {
    digitalWrite(TXLED, LOW);
  }
  if (re_pointer == (int) (SIZE * 2 / 3)) {
    digitalWrite(RXLED, LOW);
  }
  if (re_pointer == SIZE) {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(TXLED, HIGH);
    digitalWrite(RXLED, HIGH);
    
    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if(envelope(out, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: the principal components are unit norm
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
        proj1 += result[i] * pca_vec1[i];
        proj2 += result[i] * pca_vec2[i];
        proj3 += result[i] * pca_vec3[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      proj3 -= projected_mean_vec[2];

      // Classification
      // Use the function 'l2_norm3' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      // YOUR CODE HERE
      for (int i = 0; i < 4; i++) {
        float dis = l2_norm3(proj1, proj2, proj3, centroids[i]);
        if (dis < best_dist) {
          best_dist = dis;
          best_index = i;
        }
      }
      
      // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      // YOUR CODE HERE
      if (best_dist < EUCLIDEAN_THRESHOLD) {
        Serial.println(best_index);
        Serial.println(best_dist);
      } else {
        Serial.println("NOT A WORD");
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
   } else {
     Serial.println("Below LOUDNESS_THRESHOLD.");
   }

    delay(2000);
    re_pointer = 0;
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void reset_blinker(void) {
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  digitalWrite(RXLED, HIGH);
  delay(100);
  digitalWrite(RXLED, LOW);
  digitalWrite(TXLED, HIGH);
  delay(100);
  digitalWrite(RXLED, HIGH);
  digitalWrite(TXLED, LOW);
  delay(100);
  digitalWrite(RXLED, HIGH);
  digitalWrite(TXLED, HIGH);
  delay(100);
  digitalWrite(TXLED, HIGH);
}

void envelope(int16_t* data, int16_t* data_out, int index){
  int32_t avg = 0;
  for (int i = 0; i < AVG_SIZE; i++) {
      avg += data[i];
  }
  
  avg = avg >> AVG_SHIFT;
  data_out[index] = abs(data[0] - avg);  
  
  for (int i = 1; i < AVG_SIZE; i++) {
      data_out[index] += abs(data[i] - avg);
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    Serial.println(maximum);
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres && block < SIZE_AFTER_FILTER);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}
/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/


ISR(TIMER1_COMPA_vect){//timer1 interrupt 8Khz toggles pin 13 (LED)
  if (re_pointer < SIZE) {
    digitalWrite(RXLED, LOW);
    get_re(write_arr)[re_pointer%AVG_SIZE] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}
