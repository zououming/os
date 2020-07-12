#define input1 3 // 定义uno的pin 5 向 input1 输出 1 2 控制左轮
#define input2 4 // 定义uno的pin 6 向 input2 输出
#define input3 5 // 定义uno的pin 9 向 input3 输出 3 4 控制右轮
#define input4 6 // 定义uno的pin 10 向 input4 输出

#define leftpow 2 // 左轮使能
#define rightpow 7

#define right_trig 10 // 右侧超声波
#define right_echo 11
#define left_trig 13  // 左侧超声波
#define left_echo 12

#define ultrasound_lenth 1
#define turn_delay 3000           // 转向延迟
#define adj_dis 5                 // 调整方向距离阈值
#define adj_num 3
#define turn_dis 100              // 判断拐角距离阈值
#define adj_delay 20              // 调整方向延迟
#define send_delay 30            // 接受数据等待时间

#define left 0
#define right 1

float now_distance[2] = {0};  // 左和右
float last_distance[2] = {0};
bool init_flag = true;

float turn_distance;//turn_distance's problem
int default_direction = right;
int default_trig;
int default_echo;

float dis_left[ultrasound_lenth];
float dis_right[ultrasound_lenth];

float avg(float a[ultrasound_lenth])
{
  float b = 0;
  for (int i = 0; i < ultrasound_lenth; i++)
  {
    b += a[i];
  }
  b = b / 10;
  return b;
}

void go_straight()
{
  digitalWrite(input1, HIGH); //给高电平
  digitalWrite(input2, LOW); //给低电平
  digitalWrite(input3, LOW); //给高电平
  digitalWrite(input4, HIGH); //给低电平
  analogWrite(leftpow, 100);//左 100 6V
  analogWrite(rightpow, 118);//右 116
}

void stop()
{
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
  digitalWrite(input3, LOW);
  digitalWrite(input4, LOW);
}

void Tright()//右转
{
  digitalWrite(input1, HIGH);
  digitalWrite(input2, LOW); 
  digitalWrite(input3, HIGH); 
  digitalWrite(input4, LOW); 
  analogWrite(leftpow, 160);//左 160 6V
  analogWrite(rightpow, 170);//右 170
}

void Tleft()//左转
{
  digitalWrite(input1, LOW); 
  digitalWrite(input2, HIGH); 
  digitalWrite(input3, LOW); 
  digitalWrite(input4, HIGH); 
  analogWrite(leftpow, 160);//左 100 6V
  analogWrite(rightpow, 170);//右 116
}

bool adjust_direction()
{
  float error = now_distance[default_direction] - turn_distance;
//  Serial.print("error: ");
//  Serial.print(error);
//  Serial.print("    dis: ");
//  Serial.println(turn_distance);

  if ( (error < -adj_dis && error > -turn_dis) || (now_distance[default_direction] < 5 && now_distance[default_direction] > 0) ){
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
  while (count < adj_num){
    delay(adj_delay);
    update_distance();
    if (last_distance[default_direction] - now_distance[default_direction] < 0)
      count++;
  }
  turn_distance = now_distance[default_direction];
  go_straight();
}

void Adj_right()
{
  int count = 0;
  Tright();
  while (count < adj_num){
    delay(adj_delay);
    update_distance();
    if (last_distance[default_direction] - now_distance[default_direction] < 0)
      count++;
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
  if (now_distance[left] - last_distance[left] > turn_dis)
    return 1;
  if (now_distance[right] - last_distance[right] > turn_dis)
    return 1;
  return 0;
}

void clear_buffer()
{
  char var;
  while( Serial.available() )
    var = Serial.read();
}

char Send()
{
  Serial.write("&r#");
  delay(send_delay);
}

String Uart()
{
  char a, b;
  String c;
  while (Serial.available())
  {
    a = Serial.read();
    if (a == '&')
      while (Serial.available())
      {
        b = Serial.read();
        if (b == '#')
          break;
        c += b;
      }
  }
  //    Serial.println("arduino: "+ c);
  return c;
}

void turn()
{
  String response;
  
  stop();
  do {
    Send();
    response = Uart();
  }
  while (response.length() == 0);
  

  if (response == "0") //左转
  {
    Tleft();
    delay(turn_delay);
    default_direction = right;
  }
  else if (response == "1") //右转
  {
    Tright();
    delay(turn_delay);
    default_direction = left;
  }
  else
  {
    stop();
    while(1);
  }
  
  clear_buffer();
  update_distance();
  turn_distance = now_distance[default_direction];
}

void setup() {
  Serial.begin (9600);
  //初始化各IO,模式为OUTPUT 输出模式
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);
  pinMode(right_trig, OUTPUT);
  pinMode(right_echo, INPUT);
  pinMode(left_trig, OUTPUT);
  pinMode(left_echo, INPUT);
  
  for (int i = 0; i < 10; i++)
    update_distance();
    
  go_straight();
  update_distance();
//  Serial.println(now_distance[0]);
  turn_distance = now_distance[default_direction];
}

void loop() {
  go_straight();
  update_distance();
  adjust_direction();
//  if (is_corner())
//    turn();

  //   for(int i=0;i<10;i++)
  
  //    djud2[i]=Ultrasonic(left_trig,left_echo);
}
