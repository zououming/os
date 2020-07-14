#define input1 3 // pin3 4 输出 input1 2 控制左轮
#define input2 4

#define input3 5 // pin5 6 输出 input3 4 控制右轮
#define input4 6

#define leftpow 2   // 左轮使能
#define rightpow 7  // 右轮使能

#define right_trig 10 // 右侧超声波
#define right_echo 11

#define left_trig 13  // 左侧超声波
#define left_echo 12

#define turn_left_delay 850       // 转向延迟
#define turn_right_delay 720
#define adj_dis 2               // 调整方向距离阈值
#define adj_num 1                 // 调整方向条件
#define adj_delay 30              // 调整方向延迟
#define turn_dis 50               // 判断拐角距离阈值
#define send_delay 50             // 接受数据等待时间
#define corner_delay 250          // 进入拐角的延迟 
#define after_turn_stop_delay 100 
#define after_turn_delay 950      // 转向后延迟

#define left 0
#define right 1

float now_distance[2] = {0};  // 左和右
float last_distance[2] = {0};
bool init_flag = true;

float turn_distance;
int default_direction = left;
int default_trig;
int default_echo;

String response;

void go_straight()
{
  digitalWrite(input1, HIGH); //给高电平
  digitalWrite(input2, LOW); //给低电平
  digitalWrite(input3, LOW); //给高电平
  digitalWrite(input4, HIGH); //给低电平
  analogWrite(leftpow, 100);//左 100 6V
  analogWrite(rightpow, 116);//右 116
}

void stop()
{
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
  digitalWrite(input3, LOW);
  digitalWrite(input4, LOW);
}

void Tleft()
{
  digitalWrite(input1, LOW);
  digitalWrite(input2, HIGH); 
  digitalWrite(input3, LOW); 
  digitalWrite(input4, HIGH); 
}

void Tright()
{
  digitalWrite(input1, HIGH); 
  digitalWrite(input2, LOW); 
  digitalWrite(input3, HIGH); 
  digitalWrite(input4, LOW); 
}

bool adjust_direction()
{
  float error = now_distance[default_direction] - turn_distance;
//  float error = now_distance[left] - now_distance[right];
//  if (is_corner()) {
//    turn();
//  }
  if (now_distance[left] < adj_dis + 1 || now_distance[left] == -1)
    Adj_right();
  else if (now_distance[right] < adj_dis + 1 || now_distance[right] == -1) 
    Adj_left();
  else if (error < -adj_dis && error > -adj_dis - 10){
    if (default_direction == right)
      Adj_left();
    else if (default_direction == left)
      Adj_right();
  }
  else if (error > adj_dis && error < adj_dis + 10){
    if (default_direction == right)
      Adj_right();
    else if (default_direction == left)
      Adj_left();
  }
}

void Adj_left()
{
  int count = 0;
  Tleft();
  analogWrite(leftpow, 100);//左 160 6V
  analogWrite(rightpow, 110);//右 170
//  delay(adj_delay);
  while (count < adj_num){
    delay(adj_delay);
    update_distance();
    if (is_corner()) {
      go_straight();
      turn();
    }
    if (last_distance[default_direction] - now_distance[default_direction] < 0)
      count++;
    else if (last_distance[default_direction] - now_distance[default_direction] > turn_dis)
      break;
  }
  turn_distance = now_distance[default_direction];
  go_straight();
}

void Adj_right()
{
  int count = 0;
  Tright();
  analogWrite(leftpow, 100);//左 160 6V
  analogWrite(rightpow, 110);//右 170
//  delay(adj_delay);
  while (count < adj_num){
    delay(adj_delay);
    update_distance();
    if (is_corner()) {
      go_straight();
      turn();
    }
    if (last_distance[default_direction] - now_distance[default_direction] < 0)
      count++;
    else if (last_distance[default_direction] - now_distance[default_direction] > turn_dis)
      break;      
  }
  turn_distance = now_distance[default_direction];
  go_straight();
}

void update_distance()
{
  last_distance[left] = now_distance[left];
  last_distance[right] = now_distance[right];
  now_distance[left] = Ultrasonic(left_trig, left_echo);
  now_distance[right] = Ultrasonic(right_trig, right_echo);
  
//  Serial.print(now_distance[left]);
//  Serial.print("    ");
//  Serial.println(now_distance[right]);
  if (init_flag)
  {
    now_distance[left] = Ultrasonic(left_trig, left_echo);
    now_distance[right] = Ultrasonic(right_trig, right_echo);
    last_distance[left] = now_distance[left];
    last_distance[right] = now_distance[right];
    init_flag = false;
  }
  
  if (default_direction == right)
  {
    default_trig = right_trig;
    default_echo = right_echo;
  }
  else
  {
    default_trig = left_trig;
    default_trig = left_echo;
  }
}

float Ultrasonic(int Trig, int Echo)
{
  float cm, temp;
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  temp = float(pulseIn(Echo, HIGH));
  cm = (temp * 1 ) / 58;
//  if (cm > 1000)
//    cm = -1;

  //  Serial.print(Echo);
  //  Serial.println();
  //  Serial.print(temp);
  //  Serial.print(" | | Distance = ");
  //  Serial.print(cm);
  //  Serial.println("cm");
  return cm;
}

bool is_corner()
{
  if (now_distance[left] >= turn_dis || now_distance[right] >= turn_dis)
    return 1;
    
  return 0;
}

char Send()
{
  Serial.write("r");
  delay(send_delay);
}

void clear_buffer()
{
  while (Serial.available())
    Serial.read();
}

String Uart()
{
  char a, b;
  String c = "";

  while (Serial.available())
  {
    a = Serial.read();
    if (a == '&'){
      while (Serial.available())
      {
        b = Serial.read();
        if (b == '#'){
          Serial.flush();
          return c;
        }
        c += b;
      }
    }
  }

  return "";
}

void turn()
{ 
  response = Uart();
  
  if (response == "2" || response == "3")
  {
    while(1)
      stop();
  }
  
  delay(corner_delay);
  
  stop();
  while (response.length() == 0){
    delay(send_delay);
    response = Uart();
  }
  
  if (response == "0") //左转
  {
    Tleft();
    analogWrite(leftpow, 160);//左 160 8V
    analogWrite(rightpow, 170);//右 170
    delay(turn_left_delay);
    default_direction = right;
  }
  else if (response == "1") //右转
  {
    Tright();
    analogWrite(leftpow, 160);//左 160 8V
    analogWrite(rightpow, 170);//右 170
    delay(turn_right_delay);
    default_direction = left;
  }
  else if (response == "2" || response == "3")
  {
    while(1)
      stop();
  }
  
  stop();
  delay(after_turn_stop_delay);
 
  Send();   
  clear_buffer();
  
  go_straight();
  delay(after_turn_delay);
  update_distance();
  turn_distance = now_distance[default_direction];
}

void setup() {
  Serial.begin (9600);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);
  pinMode(right_trig, OUTPUT);
  pinMode(right_echo, INPUT);
  pinMode(left_trig, OUTPUT);
  pinMode(left_echo, INPUT);
  
  for (int i = 0; i < 3; i++)
    update_distance();
  Send();
  go_straight();
  update_distance();
  turn_distance = now_distance[default_direction];
}

void loop() {
  go_straight();
  update_distance();
  if (is_corner())
    turn();
  else adjust_direction();
  
//   Tright();
//   delay(turn_delay);
//   analogWrite(leftpow, 160);//左 160 6V
//   analogWrite(rightpow, 170);//右 170
//   stop();
//   delay(after_turn_delay);
}
