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

#define turn_left_delay 750            // 转向延迟
#define turn_right_delay 800
#define adj_dis 2                 // 调整方向距离阈值
#define adj_num 1                 // 调整方向条件
#define adj_delay 20              // 调整方向延迟
#define turn_dis 30               // 判断拐角距离阈值
#define send_delay 30             // 接受数据等待时间
#define corner_delay 250          // 进入拐角的延迟 
#define after_turn_stop_delay 100 
#define after_turn_delay 1000      // 转向后延迟

#define left 0
#define right 1

float now_distance[2] = {0};  // 左和右
float last_distance[2] = {0};
bool init_flag = true;

float turn_distance;
int default_direction = left;
int default_trig;
int default_echo;

String receive_frame;

void go_straight()
{
  digitalWrite(input1, HIGH); //给高电平
  digitalWrite(input2, LOW); //给低电平
  digitalWrite(input3, LOW); //给高电平
  digitalWrite(input4, HIGH); //给低电平
  analogWrite(leftpow, 80);//左 100 6V
  analogWrite(rightpow, 90);//右 116
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
//  Serial.print("error: ");
//  Serial.print(error);
//  Serial.print("    dis: ");
//  Serial.println(turn_distance);

  if ( (error < -adj_dis && error > -turn_dis) || (now_distance[default_direction] < 3 && now_distance[default_direction] > 0) ){
    if (default_direction == right)
      Adj_left();
    else if (default_direction == left)
      Adj_right();
  }
  else if (error > adj_dis && error < turn_dis){
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
  while (count < adj_num){
    delay(adj_delay);
    update_distance();
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
  while (count < adj_num){
    delay(adj_delay);
    update_distance();
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
  if (cm > 300)
    cm = -1;

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
//  if (now_distance[left] == -1 || now_distance[right] == -1)
//    return 1;
  if (now_distance[left] - last_distance[left] >= turn_dis)
    return 1;
  if (now_distance[right] - last_distance[right] >= turn_dis)
    return 1;
    
  return 0;
}

char Send()
{
  Serial.write("&r#");
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
  String c;

  while (Serial.available())
  {
    a = Serial.read();
    if (a == '&'){
      receive_frame = "";
      while (Serial.available())
      {
        b = Serial.read();
        if (b == '#')
          break; 
        c += b;
      }
      Serial.flush();
//      clear_buffer();
    }
  }
   
  return c;
}

void turn()
{
  String response;
  delay(corner_delay);
  stop();
  do {
    Send();
    response = Uart();
  }
  while (response.length() == 0);
  
  if (response == "0") //左转
  {
    Tleft();
    analogWrite(leftpow, 160);//左 160 6V
    analogWrite(rightpow, 170);//右 170
    delay(turn_left_delay);
    default_direction = right;
  }
  else if (response == "1") //右转
  {
    Tright();
    analogWrite(leftpow, 160);//左 160 6V
    analogWrite(rightpow, 170);//右 170
    delay(turn_right_delay);
    default_direction = left;
  }
  else// if (response == "2" || response == "3")
  {
    while(1)
      stop();
  }
  stop();
  delay(after_turn_stop_delay);
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
  
  for (int i = 0; i < 5; i++)
    update_distance();
    
  go_straight();
  update_distance();
  turn_distance = now_distance[default_direction];
}

void loop() {
  go_straight();
  update_distance();
//  Uart();
  if (is_corner())
    turn();
  adjust_direction();

//   Tright();
//   delay(turn_delay);
//   analogWrite(leftpow, 160);//左 160 6V
//   analogWrite(rightpow, 170);//右 170
//   stop();
//   delay(after_turn_delay);
}
