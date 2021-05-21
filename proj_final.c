#include<P18F4550.h>
#include<delays.h>
#include "lcd_picsim.c"
#include<timers.h>

//---------------------------------CONFIGURACOES------------------------------------------
#pragma config	CPUDIV = OSC1_PLL2	// Fosc -> oscilador
#pragma config	FOSC = HS			// Fosc = 20MHz -> Tcy = 200ns
#pragma config	WDT = OFF			// Watchdog desativado
#pragma config	PBADEN = OFF		// Pinos do PORTB como digitais
#pragma config	DEBUG = ON			// habilita debug


void ajusta_dc2(unsigned int valor2);
void ajusta_dc(unsigned int valor_dc);

//----------------------------------------BOTOES-------------------------------------------
#define BT2 	PORTBbits.RB2
#define BT1		PORTBbits.RB1

//---------------------------------------VARIAVEIS-----------------------------------------
//variaveis de acesso aos menus, primario e secundario
char  
	confmenu=0,       	//confirmacao do menu principal
	confmenusec=0, 		//confirmacao do menu secundario
	timer1=0,			//variavel de comparacao timer decrescente
	tmp,
	temperatura[8];

//variaveis de confirmacao dos menus, sem prioridade
int
	BT1apertado = 0, 
	BT2apertado = 0, 
	BT0apertado = 0;

//variaveis de acesso ao menu secundario
char 
	set = 0, 			//cada posicao do switch
	iniciar=0,			//variavel para confirmacao de iniciar o forno
	monito=0,			//variavel para ativar monitoramento/serial
	vap=0,				//variavel para ativar vaporizacao
	Tem2[4];			//guarda a temperatura em vetor para USART e Display

int 
	temp1 = 50,			//temperatura de inicio do forno, setado em 50 graus
	seg  = 15, 			//tempo inicial do forno, 0 segundos
	min  = 0,			//10 minutos
	hora = 0,			//0 horas
	aux,				//responsavel por controlar a acao de parada e execucao do botao
	dc2;				//responsavel pelo valor do ciclo
	
long int
	Temp,				//temperatura atual do sensor
	valor_Temp;			//variavel conversao a/d

//----------------------------VARIAVEIS CONTROLADOR PI-------------------------------------

double   
	error_meas,		//diferenca do setpoint para a temperatura atual
    kp,				//constante proporcional
    ki,				//constante integrativa
	pk_1=0,			//guarda o ultimo valor do integral
    proportional,
    integral,
    PI,				//sinal de controle
    ideal_value,	//guarda valor de referencia
	
	real_temp=0,
    erro_anterior=0,
    u_anterior=0;

int  
	measure,		//valor medido
    pwm = 128;


unsigned int valor2=200; //ciclo ativo do cooler

//----------------------------------CONVERSAO A/D------------------------------------------

void lerTemp(void) {
    ADCON0bits.CHS = 0b0011;					//seleciona o canal
    ADCON0bits.GO_DONE = 1; 					//ativa a requisicao
    while (ADCON0bits.GO_DONE);
    valor_Temp = (ADRESH * 256) + ADRESL; 		//leitura da conversao
    Temp = (50000 * (long) valor_Temp) / 1023;	//faz a conversao pela tensao
    Delay10KTCYx(25); 
}

//---------------------------------CONTROLE PI---------------------------------------------
void CONTROLE_PI(){	
	kp = 48.5;								//constantes Kp e Ki
	ki = 1.5;
	
	ideal_value = temp1; 					//valor de setpoint
	measure = Temp/100;						//valor medido pelo sensor
	error_meas = ideal_value - measure;		//erro entre valor medido e o desejado
    proportional = error_meas * kp;			//proporcional
    integral = (error_meas*ki) + pk_1;		//integral
    PI = proportional + integral;			//sinal de controle
    PI = PI/4;								
    pwm = PI + 128;							
  
	if(pwm > 1024 ) {
    pwm = 1023;   }
    if(pwm < 0 ) {
    pwm = 0;
    }

    pk_1 = integral;					//salva o ultimo valor
    ajusta_dc(pwm);						//chama a funcao
  	INTCONbits.GIEL = 1;				//ativa interrupcoes de baixa prioridade
	Delay10KTCYx(100);
	Delay10KTCYx(100);
	Delay10KTCYx(100);
	Delay10KTCYx(100);
	}

//-----------------------------------DUTY CYCLE---------------------------------------------

void ajusta_dc2(unsigned int valor_dc2){		//velocidade do cooler
	CCPR1L = (char)(valor_dc2 >> 2);			//tira os dois bits menos significativos
	CCP1CONbits.DC1B0 = valor_dc2%2;			//utiliza o menos significativo
	valor_dc2 = valor_dc2 >> 1;					//tira o menos significativo
	CCP1CONbits.DC1B1 = valor_dc2%2;			//utiliza o da sequencia
}

void ajusta_dc(unsigned int valor_dc) {
	CCPR2L = (char)(valor_dc >> 2);				//tira os dois bits menos significativos
	CCP2CONbits.DC2B0 = valor_dc % 2;			//utiliza o menos significativo
	valor_dc = valor_dc >> 1;					//tira o menos significativo
	CCP2CONbits.DC2B1 = valor_dc % 2;			//utiliza o da sequencia
}


void Tempatualizado(void) {                                
    WriteCmdXLCD(0x80);	                		//posiciona na primeira linha
	temperatura[0] = 0x30+((Temp/1000)%10);
	temperatura[1] = 0x30+((Temp/100)%10);
	temperatura[2] = '.';
	temperatura[3] = 0x30+((Temp/10)%10);
	temperatura[4] = 0x30+((Temp/1)%10);
	temperatura[5] = 'C';
	putsXLCD(temperatura);
	putrsXLCD("  ");
}

//-------------------------INTERRUPCOES ALTA PRIORIDADE----------------------------------
void ISR_Alta_Prioridade(void);
#pragma code high_vector=0x08 			//vetor de int. de alta prioridade
void int_alta(void)
{
	_asm GOTO ISR_Alta_Prioridade _endasm
}
#pragma code 
#pragma interrupt ISR_Alta_Prioridade
void ISR_Alta_Prioridade(void)
{
	if (INTCONbits.TMR0IF == 1){  		//timer 0
	   TMR0H = 0x67;					//carga inicial = 26474 = 0X676A
	   TMR0L = 0X6A; 
	   timer1 = 1;						//variavel auxliar para timer decrescente
	   INTCONbits.TMR0IF = 0;
	}
     
//botao para executar
    if (INTCONbits.INT0F == 1){   	//Caso seja pressionado o bot?o RB0 
	 INTCONbits.INT0F = 0;
	if(set!=0){
	 BT0apertado = 1;				//variavel auxiliar para os menus
	 INTCONbits.GIEL = 1;			//habilita interrupcoes de baixa prioridade
	 Delay10KTCYx(100);
	}
		
        
//botao para parar
    if(iniciar==1 && confmenusec==0){ //nao estiver no menu secundario e tiver selecionado
									// para iniciar
	 INTCONbits.TMR0IE = 0;			//desabilita TMR0
	 PORTCbits.RC2 = 0;	
	 vap = 0;
	 INTCONbits.GIEL = 0;			//desabilita as interrupcoes de baixa prioridade
	 iniciar=0;						//desativa o funcionamento do forno pelo menu
	 confmenusec=1;					//volta para o menu secundario
	 aux++;							//incrementa variavel auxliar para botao ;RB0
    }    

    if(aux==1 && timer1==1&&iniciar==1){ //nao estiver no menu secundario e tiver 
										//selecionado para iniciar
		T2CON = 0b00000111;
		aux = 0;
    }    
  }     
}

//-------------------------INTERRUPCOES BAIXA PRIORIDADE----------------------------------
void ISR_Baixa_Prioridade(void);
#pragma code low_vector=0x18
void int_baixa(void)
{
	_asm GOTO ISR_Baixa_Prioridade _endasm
}
#pragma code
#pragma interruptlow ISR_Baixa_Prioridade
void ISR_Baixa_Prioridade(void)
{
}

//-----------------------------------ROTINA DE INICIO------------------------------------
void INICIO(){    
        lerTemp(); 				//realiza a leitura da temperatura
        CONTROLE_PI(); 			//realiza o controle PID
		Tempatualizado();		//atualiza o temperatura
        
//responsavel por mostrar no display inicial e ao iniciar o forno a temporizacao 
	 WriteCmdXLCD(0x88);				//posiciona na primeira linha					
	 putcXLCD(0x30+(hora/10));			//escreve a dezena da hora
	 putcXLCD(0x30+(hora%10));			//escreve a unidade da hora
	 putrsXLCD(":");
	 putcXLCD(0x30+(min/10));			//escreve a dezena do minuto
	 putcXLCD(0x30+(min%10));			//escreve a unidade do minuto
	 putrsXLCD(":");  					
	 putcXLCD(0x30+(seg/10));			//escreve a dezena do segundo
	 putcXLCD(0x30+(seg%10));			//escreve a unidade do segundo

//responsavel pelo botao de parar o forno
        WriteCmdXLCD(0xC0);				//posiciona na segunda linha
        putrsXLCD("      Parar     ");
		WriteCmdXLCD(0x0C);				//desliga o cursor
}

//-----------------------------------ROTINA DO MENU------------------------------------
      void MENU ()
     {
            switch (set){
                case 1:
		     WriteCmdXLCD(0x80);			//posiciona na primeira linha	
		     putrsXLCD("Forno Industrial");
		     WriteCmdXLCD(0xC0);			//posiciona na segunda linha
		     putrsXLCD("1. Ajust Temper.");	           
		     break;
                case 2:
		     WriteCmdXLCD(0x80);			//posiciona na primeira linha	
		     putrsXLCD("Forno Industrial");
		     WriteCmdXLCD(0xC0);			//posiciona na segunda linha
		     putrsXLCD("2. Ajust Tempor.");		
		     break;    
                case 3:
		     WriteCmdXLCD(0x80);			//posiciona na primeira linha	
		     putrsXLCD("Forno Industrial");
		     WriteCmdXLCD(0xC0);			//posiciona na segunda linha
		     putrsXLCD("3. Ativar Vapor.");		
		     break;
	       case 4:
		     WriteCmdXLCD(0x80);			 //posiciona na primeira linha	
		     putrsXLCD("Forno Industrial");
		     WriteCmdXLCD(0xC0);			//posiciona na segunda linha
		     putrsXLCD("4. Ativar Monit.");			
		     break;
                case 5:
		     WriteCmdXLCD(0x80);			//posiciona na primeira linha	
		     putrsXLCD("Forno Industrial");
		     WriteCmdXLCD(0xC0);			//posiciona na segunda linha
		     putrsXLCD("   5. Iniciar   ");			
		     break;
            }
            confmenu=1; // variavel auxiliar para dizer que esta dentro do menu principal
}
//-------------------------ROTINA DO MENU SECUNDARIO----------------------------------
      void MENUSEC()
   {
	switch (set){
           case 1: 
			     WriteCmdXLCD(0x80);				//posiciona na primeira linha	
			     putrsXLCD("Ajustar Temperat");
			     WriteCmdXLCD(0xC0);		       	//posiciona na segunda linha
			     putrsXLCD("      ");
			     putcXLCD(0x30+(temp1/100));		//escreve a centena da temperatura
			     putcXLCD(0x30+(temp1/10)%10);		//escreve a unidade da temperatura
			     putrsXLCD("0");
			     putcXLCD(0b11011111);             	//escreve o icone de grau
			     putrsXLCD("C       ");       		//escreve C maiusculo		   		
			break;
			case 2:   
                WriteCmdXLCD(0x80);				//posiciona na primeira linha	
                putrsXLCD("Ajustar Temporiz");
                WriteCmdXLCD(0xC0);				//posiciona na segunda linha				
                putrsXLCD("    ");
  				putcXLCD(0x30+(hora/10));		//escreve a dezena da hora
                putcXLCD(0x30+(hora%10));		//escreve a unidade da hora
                putrsXLCD(":");
                putcXLCD(0x30+(min/10));		//escreve a dezena do minuto
                putcXLCD(0x30+(min%10));		//escreve a unidade do minuto
                putrsXLCD(":");  				
                putcXLCD(0x30+(seg/10));		//escreve a dezena dos segundo
                putcXLCD(0x30+(seg%10));		//escreve a unidade do segundo
                putrsXLCD("      ");
			break;
			case 3:  
                WriteCmdXLCD(0x80);					//posiciona na primeira linha
                putrsXLCD("Ativar Vaporiza.");
               	WriteCmdXLCD(0xC0);					//posiciona na segunda linha	
			      if(vap == 1){						//variavel ativa vaporizacao
				 	 putrsXLCD("Vaporiz. Ativada");}
			      if(vap == 0){						//variavel desativa vaporizacao
				  	putrsXLCD("Vapo. Desativada");}
	       break;
	       case 4:
                WriteCmdXLCD(0x80);						//posiciona na primeira linha
                putrsXLCD("Ativar Monitoram");
                WriteCmdXLCD(0xC0);						//posiciona na segunda linha			
					if(monito == 1){					//variavel ativa monitoramento
					putrsXLCD("Monitora. Ativada");}
					if(monito == 0){					//variavel desativa monitoramento	
					putrsXLCD("Monit. Desativada");}
		   break;
	       case 5:     
				WriteCmdXLCD(0x80);						//posiciona na primeira linha
				putrsXLCD(" Iniciar o Forno");
					if(iniciar==1){						//variavel inicia o forno
					WriteCmdXLCD(0xC0);					//posiciona na segunda linha	
					putrsXLCD("     Iniciar    ");}										
					if(iniciar==0){						//variavel nao inicia o forno
				WriteCmdXLCD(0xC0);
				putrsXLCD("   Nao Iniciar  ");}	
		  break;                            
	    }
            Delay10KTCYx(20);
	   		confmenusec=1; 	//variavel auxiliar para dizer que esta dentro do menu 
							//secundario
}



//----------------------------------ROTINA MAIN--------------------------------------------
void main(void){
	TRISA = 0b00001011; 	//RA1 e RA2 como entrada
    ADCON0 = 0b00001101; 	//canal 3 e A/D ligado
    ADCON1 = 0b00001011; 	//Vss e Vdd, RA0, RA1 e RA2 analogico
    ADCON2 = 0b10111101; 	//just direita, 20Tad e fosc/16
   
	TRISB = 0b00000111;					//RB0, RB1, RB2 como entrada
    TRISD = 0x00;
    TRISE = 0x00;   
    TRISC = 0b00000000;					//PORTC como saida
 
//inicializacao do lcd
    OpenXLCD();	 						//comunicacao por byte
    INTCON2bits.NOT_RBPU = 0;			//habilita pull-ups da PORTB

//interrupcoes configuracao
	RCONbits.IPEN = 1;					// habilita prioridade
    
//configuracoes int0
	INTCONbits.INT0IE = 1;				// habilita INT0
	INTCONbits.INT0IF = 0;				// zera flag da INT0
  	
//configuracao timer 0
	INTCONbits.TMR0IF = 0;				//zera flag do TMR0
	INTCON2bits.TMR0IP = 1;				//interrupcao de alta prioridade
   	INTCONbits.TMR0IE = 0;				//desabilita TMR0
	T0CON = 0b10000110;					//TMR0ON = ON, T08BIT = 16 BITS, T0CS = CLKO, X, 
	TMR0H = 0x67;						//CARGA INICIAL = 26474 = 0X676A
	TMR0L = 0X6A;						//PSA = YES, T0PS2:0 = 128

//config Registradores USART
	TXSTAbits.BRGH = 1;
	TXSTAbits.SYNC = 0;
	BAUDCONbits.BRG16 = 0;
	RCSTAbits.SPEN = 1;
	TRISCbits.TRISC6 = 0;
	TXSTAbits.TXEN = 1;
	SPBRG = 129;
	SPBRGH = 0;

//config TIMER2, CCP1 e CCP2
    T2CON = 0b00000111; // Timer2 on, prescaler = 16, postscaler n?o importa
    PR2 = 249; // Contagem m?xima (carga inicial zero),
    CCP2CON = 0b00001100; // Modo PWM no CCP2

  	CCP1CON = 0b00001100;	// Modo PWM 
							// CCP1M3:0 = 1100 => active high
							// CCP1M3:0 = 1111 => active low

//configuracao timer 1
 	PIE1bits.TMR1IE = 1;
    PIR1bits.TMR1IF = 0;
    IPR1bits.TMR1IP = 0;
    T1CON = 0b10110000;

    TMR1H = 0x85;		//cargas
    TMR1L = 0xEE;

    PIE2bits.TMR3IE = 1;
    PIR2bits.TMR3IF = 0;
    IPR2bits.TMR3IP = 0;
    T3CON = 0b10110000;

    TMR3H = 0x85;
    TMR3L = 0xEE;

    INTCON2bits.INTEDG1 = 1;
    INTCON2bits.INTEDG1 = 1;
    INTCONbits.RBIE = 0;

//habilitacao interrupcoes de alta e baixa prioridade   
	INTCONbits.GIEL = 1;				//ativa interrupcoes de baixa prioridade
	INTCONbits.GIEH = 1;				//ativa interrupcoes de alta prioridade

 	OpenXLCD();							//comunicacao por byte
	WriteCmdXLCD(0x0C);	    			//desabilita cursor do display
    

//----------------------------------ROTINA MAIN--------------------------------------------
		
	INICIO();  	//inicia o firmware apresentando o menu prinicipal com temp e tempo definido
				//inicialmente

	while(1) {	
	char dc2_str[3];
	char dc2;
	ajusta_dc2(0);		//inicia cooler desligado
	ajusta_dc(0);

	//envio para a serial USART
		if(monito==1 && confmenusec==0 && iniciar==1)
		for (tmp=0;tmp<=5;tmp++){
			while(TXSTAbits.TRMT == 0){};
			TXREG = temperatura[tmp];
		}
	   	while(TXSTAbits.TRMT == 0){};
	   	TXREG = '\n';		
       	while(TXSTAbits.TRMT == 0){};
	   	TXREG = '\r';

	//acionar Vaporizacao
		if(vap==1 && confmenusec==0 && iniciar==1)
		ajusta_dc2(valor2);		//inicia vaporizacao
	
	if(confmenusec==0 && iniciar==1){	//Neste caso nao esta no menu sec e foi clicado para iniciar
	INICIO();  							// apresenta a fun??o principal com temperatura e tempo
	INTCONbits.TMR0IE = 1;				//inicia o timer0, dando inicio ao fornp
}

//---------------------------CONFIGURACAO BOTOES-----------------------------------

        if(BT1==0){
		BT1apertado = 1;		//define como uma variavel para facilitar as comparacoes
		Delay10KTCYx(50);}
		if(BT2==0){
		BT2apertado = 1;;		//define como uma variavel para facilitar as comparacoes
		Delay10KTCYx(50);}
		
        if(BT1apertado == 1 && confmenusec==0){    //botao seta para baixo, BT1
            Delay10KTCYx(100);
            set--;				//decrementa variavel set para trocar as opcoes do menu principal
								//e secundario
	       if(set < 1){
	                set = 5;}
	    BT1apertado = 0;
	    WriteCmdXLCD(0x01);					
	    MENU();							//apresenta a funcao menu
        }
        
        if(BT2apertado == 1 && confmenusec==0){  //botao seta para cima, BT2
           Delay10KTCYx(100);  
	   set++;							//incrementa variavel set para trocar as opcoes do menu 
										//principal e secundario
 	    if(set > 5){
	                set = 1;}
	    BT2apertado = 0;
	    WriteCmdXLCD(0x01);					
	    MENU();							//apresenta a funcao menu
        }

//------------------------------------MENU DE AJUSTES-------------------------------------------

        if(BT1apertado == 1 && confmenusec==1){   
		switch(set){
		     case 1: 
			   temp1=temp1-10;			//idecrementa 10 graus da temperatura no ajuste
			   if (temp1<50){
			   temp1=250;}
				 if(temp1>250){			//caso temperatura passe de 250, seta para 50
				    temp1=50;}
			   break;
		     case 2:
			   seg=0;
			   min=min-10;				//idecrementa 10 minutos no ajuste
				 if (min<10&&hora==0){     
				    min = 60;
				    hora=1;}
				 if (min<=0){    		//caso minuto chegue a zero, seta para 60
				    hora=0;
				    min=60;}
			   break;
		     case 3:
			      if(vap==1){
				 vap=0;					//desativa vaporizacao
			   break;}	
			      if(vap==0){
				 vap=1;					//ativa vaporizacao
			   break;}
		     case 4:
			      if(monito==1){
				    monito=0;			//desativa monitoramento
			      break;}
			      if(monito==0){
				    monito=1;			//ativa monitoramento
			      break;}	
		     case 5:
			      if(iniciar==1){
				    iniciar=0;			//nao inicia o forno
			      break;}
			      if(iniciar==0){
				    iniciar=1;			//inicia o forno
			      break;}
			}
			MENUSEC();					//volta para o menu secundario
			BT1apertado = 0;
		}
    
        if(BT2apertado == 1 && confmenusec==1){ 
		switch(set){
		     case 1:
		     temp1=temp1+10;			//incrementa 10 graus da temperatura no ajuste
			      if(temp1>250){	
				 temp1=50;}				//caso temperatura passe de 250, seta para 50
			      if (temp1<50){
				 temp1=250;}			//caso temperatura for menor que 50, seta para 250
			     break;
		     case 2:
		     min=min+10;
			   if (min >= 60){  			//incrementa 10 minutos no ajuste
			      min = 0;
			      hora++;}
			   if (hora == 2 && min>0 ){  	//caso hora chegue a 2, seta para 0
			      hora = 0;}
			   break;
		     case 3:
			      if(vap==1){
				 vap=0;					//desativa vaporizacao
			   break;}
			      if(vap==0){
				 vap=1;					//ativa vaporizacao
			   break;}	
		     case 4:
			      if(monito==1){
				    monito=0;			//desativa monitoramento
			      break;}
			      if(monito==0){
				    monito=1;			//ativa monitoramento
			      break;}	
		     case 5:
			      if(iniciar==1){
				    iniciar=0;			//nao inicia o forno
			      break;}
			      if(iniciar==0){
				    iniciar=1;			//inicia o forno
			      break;}
			}
			MENUSEC();					//volta para o menu secundario
			BT2apertado = 0;
        }

//---------------------------------CONDICOES PARA AJUSTE-------------------------------------- 
     if(BT0apertado == 1 && confmenusec==0 ){           
         BT0apertado = 0;
 		 MENUSEC();					//fica no menu secundario
	 }
	if(BT0apertado == 1 && confmenusec==1){           
        BT0apertado = 0;
	    confmenusec= 0;				//sai do menu secundario
	    WriteCmdXLCD(0x01);	
	    MENU();						//volta para o menu
	 }  

//----------------------------------TIMER DECRESCENTE------------------------------------------
     
		if (timer1==1&&iniciar==1){
		     seg = seg - 1;				//decrementa o segundo para mostrar no lcd
		if(seg>60){						//caso segundo seja maior que 60, seta em zero
		     seg=0;}
		if(seg<=0&&min>0){				//se segundos for <= a zero e minuto maior que 0
		     min--;						//decrementa o minuto para mostrar no lcd
		     seg=59;}
		if(seg==0&&min==0&&hora>=1){
			min=59;
			seg=59;
			hora--;}					//decrementa a hora para mostrar no lcd
			timer1=0;
		}
		
//-------------------------------------TIMER ZERADO-------------------------------------------
		if(seg<=0 && hora<=0 && min<=0){
		INTCONbits.TMR0IE = 0;			//desativa o timer 0
		seg=0;							//zera os segundos
		monito=0;
		ajusta_dc2(0);					//desativa vaporizacao
		INTCONbits.GIEL = 0;			//desabilita as interrupcoes de baixa prioridade
		T2CON = 0b00000000;
	}
}
}

    