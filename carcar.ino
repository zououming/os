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

#define turn_left_delay 440        // 转向延迟
#define turn_right_delay 390
#define adj_dis 1.2                // 调整方向距离阈值
#define adj_num 2                  // 调整方向条件
#define adj_delay 30               // 调整方向延迟
#define min_dis 2
#define max_dis 300
#define turn_dis 100               // 判断拐角距离阈值
#define send_delay 50              // 接受数据等待时间
#define corner_delay 270           // 进入拐角的延迟 
#define after_turn_stop_delay 100
#define after_turn_delay 800      // 转向后延迟
#define stuck_delay 100

#define left 0
#define right 1

//#define SHOW_DIS

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

bool is_corner()
{
  if (now_distance[left] >= turn_dis || now_distance[right] >= turn_dis)
    return 1;

  return 0;
}

void adjust_direction()
{
  float error = now_distance[default_direction] - turn_distance;
  update_distance();
  if (is_corner())
    turn();
  else if (now_distance[left] < 0 || now_distance[right] < 0)
  {
    go_straight();
    analogWrite(leftpow, 200);//左 100 6V
    analogWrite(rightpow, 200);//右 116
    delay(stuck_delay);
  }
  else if (now_distance[left] < min_dis)
    Adj_right();
  else if (now_distance[right] < min_dis) 
    Adj_left();
  else if (error < -adj_dis){
    if (default_direction == right)
      Adj_left();
    else if (default_direction == left)
      Adj_right();
  }
  else if (error > adj_dis){
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
  analogWrite(leftpow, 120);//左 160 6V
  analogWrite(rightpow, 130);//右 170

  while (count < adj_num){
    delay(adj_delay);
    update_distance();
    if (is_corner()) {
      go_straight();
      turn();
    }
    if (last_distance[default_direction] - now_distance[default_direction] < 0)
      count++;
  }
  update_distance();
  turn_distance = now_distance[default_direction];
  go_straight();
}

void Adj_right()
{
  int count = 0;
  Tright();
  analogWrite(leftpow, 120);//左 160 6V
  analogWrite(rightpow, 130);//右 170

  while (count < adj_num){
    delay(adj_delay);
    update_distance();
    if (is_corner()) {
      go_straight();
      turn();
    }
    if (last_distance[default_direction] - now_distance[default_direction] < 0)
      count++;
  }
  update_distance();
  turn_distance = now_distance[default_direction];
  go_straight();
}

void update_distance()
{
  last_distance[left] = now_distance[left];
  last_distance[right] = now_distance[right];
  now_distance[left] = Ultrasonic(left_trig, left_echo);
  now_distance[right] = Ultrasonic(right_trig, right_echo);

#ifdef SHOW_DIS
  Serial.print(now_distance[left]);
  Serial.print("    ");
  Serial.println(now_distance[right]);
#endif
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
  cm = temp  / 58;
  if (cm > max_dis)
    cm = -1;

  return cm;
}



void Send(String message)
{
  message += "\n";
  const char* send_message = message.c_str();
  Serial.write(send_message);
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
        if (b == '#')
          return c;
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
    Send("end");
    while(1)
      stop();
  }
  
  delay(corner_delay);
  stop();
  
  do{
    response = Uart();
    delay(send_delay);
  }
  while (response.length() == 0);
 
  if (response == "0") //左转
  {
    Tleft();
    analogWrite(leftpow, 255);//左 160 8V
    analogWrite(rightpow, 255);//右 170
    delay(turn_left_delay);
    default_direction = left;
  }
  else if (response == "1") //右转
  {
    Tright();
    analogWrite(leftpow, 255);//左 160 8V
    analogWrite(rightpow, 255);//右 170
    delay(turn_right_delay);
    default_direction = right;
  }
  else if (response == "2" || response == "3")
  {
    Send("end");
    while(1)
      stop();
  }
  else
  {
    while(1)
      Tright();
  }
  
  stop();
  Send("end");
  delay(after_turn_stop_delay);

  go_straight();
  delay(after_turn_delay);

  clear_buffer();

  update_distance();
  turn_distance = now_distance[default_direction];
}

void setup() {
  Serial.end();
  delay(100);
  Serial.begin (9600);
  Send("end");
  
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
  go_straight();
  update_distance();
  turn_distance = now_distance[default_direction];
}


#ifdef SHOW_DIS
void loop() {
  update_distance();
}
#else
void loop() {
  go_straight();
  update_distance();
  if (is_corner())
    turn();
  else 
    adjust_direction();
}
#endif
