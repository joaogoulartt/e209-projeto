#include <Arduino.h>

#define FOSC 16000000U // ClockSpeed
#define BAUD 9600
#define MYUBRR FOSC / 16 / BAUD - 1

#define SENSOR (1<<PD2)
#define MOTOR (1<<PD6)
#define ALARME (1<<PD7)

char msg_tx[20];
char msg_rx[32];
uint8_t pos_msg_rx = 0;
uint8_t tamanho_msg_rx = 3;
unsigned int volume = 300;
unsigned int tempo = 180;
float ideal_flow = 0;
float real_flow = 0;
int motor_power = 0;
int btn_cont = 0;
int cont = 0;
int total_cont = 0;

void UART_config(unsigned int ubrr) {
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  // Habilita a recepcao, tranmissao e interrupcao na recepcao
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  // Configura o formato da mensagem: 8 bits de dados e 1bits de stop
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_transmit(char* dados) {
  // Envia todos os caracteres do buffer dados ate chegar um final de linha
  while (*dados != 0)
  {
    // Aguarda a transmissão acabar
    while (!(UCSR0A & (1 << UDRE0)))
      ;
    // Escreve o caractere no registro de tranmissão
    UDR0 = *dados;
    // Passa para o próximo caractere do buffer dados
    dados++;
  }
}

void UART_printFloat(float value) {
  char buffer[20];
  dtostrf(value, 0, 2, buffer);
  UART_transmit(buffer);
}

ISR(USART_RX_vect) {
  // Le o dado recebido e coloca no buffer
  msg_rx[pos_msg_rx++] = UDR0;
  // Se o dado recebido for um final de linha, seta a flag de string recebida
  if (pos_msg_rx == tamanho_msg_rx)
  {
    pos_msg_rx = 0;
  }
}

void ADC_config(void)
{
  // Configurando Vref para VCC = 5V
  ADMUX = (1 << REFS0);
  /*
    ADC ativado e preescaler de 128
    16MHz / 128 = 125kHz
    ADEN = ADC Enable, ativa o ADC
    ADPSx = ADC Prescaler Select Bits
    1 1 1 = clock / 128
  */
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

int ADC_read(u8 ch)
{
  char i;
  int ADC_temp = 0; // ADC temporário, para manipular leitura
  int ADC_read = 0; // ADC_read
  ch &= 0x07;
  // Zerar os 3 primeiros bits e manter o resto
  ADMUX = (ADMUX & 0xF8) | ch;
  // ADSC (ADC Start Conversion)
  ADCSRA |= (1 << ADSC); // Faça uma conversão
  // ADIF (ADC Interrupt Flag) é setada quando o ADC pede interrupção
  // e resetada quando o vetor de interrupção
  // é tratado.
  while (!(ADCSRA & (1 << ADIF)))
    ;                   // Aguarde a conversão do sinal
  for (i = 0; i < 8; i++) // Fazendo a conversão 8 vezes para maior precisão
  {
    ADCSRA |= (1 << ADSC); // Faça uma conversão
    while (!(ADCSRA & (1 << ADIF)))
      ;                    // Aguarde a conversão do sinal
    ADC_temp = ADCL;         // lê o registro ADCL
    ADC_temp += (ADCH << 8); // lê o registro ADCH
    ADC_read += ADC_temp;    // Acumula o resultado (8 amostras) para média
  }
  ADC_read = ADC_read >> 3; // média das 8 amostras ( >> 3 é o mesmo que /8)
  return ADC_read;
}

float get_voltage(u8 ch) {
  u16 adc_result;
  adc_result = ADC_read(ch);
  float m = -5 / 1023.0;
  float b = 5.0;

  return m * adc_result + b;
}

void PWM_config() {
  TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A1); // Modo 7: Fast PWM, TOP = 0xFF, Clear OC0A on Compare Match, set OC0A at BOTTOM
  TCCR0B |= (1 << CS00);                                 // Prescaler 1: No prescaling

  OCR0A = 0; // Inicia com 0% de duty cycle
}

void INT_config() {
  EICRA = (1 << ISC01) | (1 << ISC00); // Configura a interrupção externa para borda de subida
  EIMSK = (1 << INT0);                 // Habilita a interrupção externa
}

void TIMER_config() {
  TCCR2A = (1 << WGM21); // Configuração do modo de funcionamento para Comparador
  TCCR2B = (1 << CS21);  // Pre-scaler de 8 (Frequência de 2MHz - Período de 500 ns em cada contagem)
  OCR2A = 199;           // 200 contagens de 500 ns, o que gera uma interrupção a cada 100 us 
}

void TIMER_start()
{
    TIMSK2 = (1 << OCIE2A); // Gerar uma interrupção no estouro do comparador A
}

void TIMER_stop()
{
    TIMSK2 = 0; // Desabilita a interrupção do Timer2
    total_cont = cont;
    cont = 0;
    get_error();
}

void alarmState() {
  float voltage = get_voltage(ADC0D);
  if (voltage < 0.5) {
    PORTD |= ALARME;
    PORTD &= ~MOTOR;
    UART_transmit("Bolha na tubulacao \n");
  }
  else {
    PORTD &= ~ALARME;
  }
}

void get_ideal_flow() {
  ideal_flow = volume / (tempo / 60.0);
}

void get_real_flow(){
  float seconds = total_cont / 10;
  real_flow = (2/(seconds/3600)) * 0.05;
}

void get_error(){
  get_real_flow();
  float error = ((real_flow - ideal_flow)/ideal_flow)*100.00;
  UART_transmit("Erro: ");
  UART_printFloat(error);
  UART_transmit("\n");
}

void setup_motor() {
  get_ideal_flow();
  float power_percentage;

  power_percentage = (ideal_flow / 450) * 100.0;

  motor_power = int(power_percentage * 255);
}

int main() {
  DDRD = MOTOR + ALARME;

  PORTD &= ~(MOTOR + ALARME);

  UART_config(MYUBRR);
  ADC_config();
  PWM_config();
  INT_config();
  TIMER_config();
  sei();

  while (1)
  {
    setup_motor();
    OCR0A = motor_power;
    alarmState();
    _delay_ms(300);
  }
  return 0;
}

ISR(INT0_vect) {
  if(btn_cont == 0){
    UART_transmit("CLICK 0");
    btn_cont++;
    //TIMER_start();
  }
  else{
    UART_transmit("CLICK 1");
    btn_cont--;
    //TIMER_stop();
  }
}

ISR(TIMER2_COMPA_vect) // Rotina de interrupção do Timer 2
{
    cont++;
}