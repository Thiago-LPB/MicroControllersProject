;--------------------------------------------------------------------------------
; DEFINIÇÕES DE REGISTRADORES
;--------------------------------------------------------------------------------

.def temp  = r16			;Registrador temporário
.def count = r17			;Unidade
.def tens  = r18			;Dezena
.def temp_reg = r20			;Registrador temporário auxiliar, usado em operações de I/O 
;--------------------------------------------------------------------------------
; SEGMENTO DE CÓDIGO (Memória FLASH)
;--------------------------------------------------------------------------------
.cseg
.org 0x0000					; Define a posição original do código na memória Flash (endereço 0)
    rjmp reset				; Instrução de salto relativo para a rotina de inicialização (reset)

; VETOR DE INTERRUPÇÃO DO TIMER1
.org 0x0016				; Define a posição do vetor de interrupção para "Timer1"
    rjmp timer1_isr		; Quando a interrupção do Timer1 ocorrer, o programa pulará para a rotina 'timer1_isr'.

;----------------------------------------
; Memória SRAM para semáforos
;----------------------------------------
; A diretiva .dseg inicia a definição de variáveis que serão armazenadas na memória RAM (SRAM)
.dseg
fase_praia: .byte 1		; Variável de 1 byte para armazenar a fase atual do semáforo da Praia (0=vermelho, 1=amarelo, 2=verde)
tempo_praia: .byte 1	; Variável de 1 byte para armazenar o tempo restante da fase atual do semáforo da Praia

fase_esqjati: .byte 1	; Variável para a fase do semáforo Esquerda Jatiúca
tempo_esqjati: .byte 1	; Variável para o tempo do semáforo Esquerda Jatiúca

fase_dirjati: .byte 1	; Variável para a fase do semáforo Direita Jatiúca
tempo_dirjati: .byte 1	; Variável para o tempo do semáforo Direita Jatiúca

fase_paju: .byte 1		; Variável para a fase do semáforo Pajuçara
tempo_paju: .byte 1		; Variável para o tempo do semáforo Pajuçara

flag_atualizar_serial: .byte 1  ; flag para enviar status 
								; 1= enviar, 0= não enviar

;--------------------------------------------------------------------------------
; INÍCIO DO CÓDIGO EXECUTÁVEL
;--------------------------------------------------------------------------------
.cseg

reset:
    ; --- Inicialização do Stack Pointer (Ponteiro da Pilha) ---
    ldi temp, low(RAMEND)	; Carrega a parte baixa do endereço final da RAM em 'temp'
    out SPL, temp			; Escreve no registrador Stack Pointer Low (SPL)
    ldi temp, high(RAMEND)	; Carrega a parte alta do endereço final da RAM em 'temp'
    out SPH, temp			; Escreve no registrador Stack Pointer High (SPH)

    ; --- Inicialização da Comunicação Serial (UART) ---
    rcall uart_init			; Chama a sub-rotina para configurar a UART com 9600 baud rate

	; --- Configuração das Portas de Saída (LEDs e Displays) ---
    ; Configura os pinos do PORTC (PC0-PC5) como saída
    ; PC0-PC3 são para os dados BCD do display de 7 segmentos
    ; PC4-PC5 são para LEDs do semáforo da Pajuçara
    ldi temp, (1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5)
    out DDRC, temp

    ; Configura os pinos do PORTB (PB0-PB5) como saída
    ; PB0-PB1 controlam a multiplexação dos displays (via transistores TIP122)
    ; PB2-PB4 controlam os LEDs do semáforo da Praia
    ; PB5 controla um LED do semáforo da Pajuçara
    ldi temp, (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB5)
    out DDRB, temp

	; Configura os pinos do PORTD (PD2-PD7) como saída para os LEDs dos outros semáforos
	ldi temp, (1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7)
	out DDRD, temp

	; --- Inicialização dos Estados e Tempos dos Semáforos ---
	; Define o estado inicial de cada semáforo
    ; Todos começam no vermelho (fase 0) com seus respectivos tempos de contagem
	ldi temp, 0
	sts fase_praia, temp	; Define a fase inicial da Praia como Vermelho (0)
	ldi temp, 95			
	sts tempo_praia, temp	; Define o tempo inicial da Praia.

	ldi temp, 0
	sts fase_esqjati, temp	; Define a fase inicial de Esq. Jatiúca como Vermelho (0)
	ldi temp, 5
	sts tempo_esqjati, temp	; Define o tempo inicial

	ldi temp, 0
	sts fase_dirjati, temp	; Define a fase inicial de Dir. Jatiúca como Vermelho (0)
	ldi temp, 5
	sts tempo_dirjati, temp	; Define o tempo inicial

	ldi temp, 0
	sts fase_paju, temp		; Define a fase inicial de Pajuçara como Vermelho (0)
	ldi temp, 31
	sts tempo_paju, temp	; Define o tempo inicial

	; Zera a flag de atualização da serial no início
    ldi temp, 0
    sts flag_atualizar_serial, temp

    ; --- Configuração do Timer1 para gerar uma interrupção a cada 1 segundo (1 Hz) ---
    ; Modo CTC (Clear Timer on Compare Match): O timer conta até o valor em OCR1A e então reinicia
    ldi temp, (1<<WGM12)
    sts TCCR1B, temp	; Configura o modo CTC no registrador de controle B

	; Define o valor de comparação para gerar a interrupção em 1 Hz
    ; Cálculo: (F_CPU / Prescaler) / F_interrupção = (16,000,000 / 1024) / 1 Hz = 15625
    ; Como a contagem vai de 0 a 15624, o valor a ser carregado é 15624
    ldi temp, high(15624)
    sts OCR1AH, temp		; Carrega a parte alta do valor de comparação
    ldi temp, low(15624)
    sts OCR1AL, temp		; Carrega a parte baixa do valor de comparação

	; Configura o prescaler (divisor de clock) para 1024 e ativa o modo CTC
    ; O prescaler reduz a velocidade do clock do timer, permitindo contar intervalos mais longos
    ldi temp, (1<<CS12)|(1<<CS10)|(1<<WGM12)
    sts TCCR1B, temp

	; Habilita a interrupção de comparação do canal A do Timer1
    ldi temp, (1<<OCIE1A)
    sts TIMSK1, temp

	; --- Habilitação Global de Interrupções ---
    sei						; Seta o bit 'I' no SREG, permitindo que as interrupções ocorram


;--------------------------------------------------------------------------------
; LOOP PRINCIPAL
;--------------------------------------------------------------------------------
main_lp:
    ; Verifica se a rotina de interrupção do timer sinalizou para enviar o status
    lds temp, flag_atualizar_serial	; Carrega o valor da flag para o registrador 'temp'
    cpi temp, 1						; Compara o valor com 1
    brne continua_main				; Se não for igual a 1, pula para 'continua_main'
   
   ; Se a flag for 1, envia o status e reseta a flag.
    rcall enviar_status_serial		; Chama a rotina que envia o status pela UART
    ldi temp, 0
    sts flag_atualizar_serial, temp	; Zera a flag para evitar envios repetidos

continua_main:
    rcall multiplex					; Chama a rotina que atualiza os displays e os LEDs dos semáforos
    rjmp main_lp					; Salta de volta para o início do loop, criando um ciclo infinito.

;--------------------------------------------------------------------------------
; ROTINAS DE COMUNICAÇÃO SERIAL (UART)
;--------------------------------------------------------------------------------

; --- Inicializa a UART ---
; Configura a comunicação serial para 9600 bits por segundo (baud rate) com um clock de 16MHz
uart_init:
    ; O valor de UBRR (USART Baud Rate Register) é calculado para a taxa desejada
    ; Para 9600 baud @ 16MHz, UBRR = 103
    ldi temp, 0
    sts UBRR0H, temp	; Parte alta do UBRR (zerada para 103)
    ldi temp, 103
    sts UBRR0L, temp	; Parte baixa do UBRR	
    
    ; Habilita o transmissor (TXEN0)
    ldi temp, (1<<TXEN0)
    sts UCSR0B, temp
    
    ; Configura o formato do frame: 8 bits de dados, 1 stop bit, sem paridade
    ldi temp, (1<<UCSZ01)|(1<<UCSZ00)
    sts UCSR0C, temp
    ret					; Retorna da sub-rotina

; --- Transmite um único byte via UART ---
; O byte a ser enviado deve estar no registrador 'temp'
uart_transmit:
    push r19				; Salva r19 na pilha para não perder seu valor.
uart_wait:
    lds r19, UCSR0A			; Lê o registrador de status A da UART
    sbrs r19, UDRE0			; Testa o bit UDRE0 (USART Data Register Empty). Pula a próxima instrução se o bit estiver setado
    rjmp uart_wait			; Se o buffer de transmissão não estiver vazio, fica em loop esperando
    sts UDR0, temp			; Quando o buffer estiver vazio, escreve o byte de 'temp' para transmissão
    pop r19					; Restaura o valor original de r19 da pilha
    ret						; Retorna da sub-rotina

; --- Envia uma string armazenada na memória Flash via UART ---
; O endereço da string deve estar no ponteiro Z (registradores ZH:ZL)
uart_send_string:
    push r19				; Salva registradores que serão usados na rotina.
    push ZL
    push ZH
uart_str_loop:
    lpm r19, Z+                     ; Carrega um byte da memória de programa (Flash) para r19 e incrementa o ponteiro Z
    cpi r19, 0                      ; Compara o byte carregado com 0 (caractere nulo, fim da string)
    breq uart_str_end               ; Se for igual a zero, salta para o final
    mov temp, r19                   ; Move o caractere para 'temp' para ser enviado
    rcall uart_transmit             ; Chama a rotina de transmissão
    rjmp uart_str_loop              ; Volta para o início do loop para pegar o próximo caractere
uart_str_end:
    pop ZH							; Restaura os valores dos registradores da pilha
    pop ZL
    pop r19
    ret

;--------------------------------------------------------------------------------
; ROTINAS DE STATUS SERIAL
;--------------------------------------------------------------------------------

; --- Envia o status completo de todos os semáforos via serial ---
enviar_status_serial:
    push ZL							; Salva o ponteiro Z
    push ZH
    
    ; Envia o cabeçalho "STATUS: "
    ldi ZL, low(2*msg_inicio)       ; Carrega o endereço da string na memória Flash para o ponteiro Z
    ldi ZH, high(2*msg_inicio)      ; O endereço é multiplicado por 2 porque a Flash é endereçada por palavra (2 bytes)
    rcall uart_send_string
    
    ; Envia o status do Semáforo Praia
    ldi ZL, low(2*msg_praia)
    ldi ZH, high(2*msg_praia)
    rcall uart_send_string
    lds temp, fase_praia            ; Carrega a fase atual do semáforo
    rcall enviar_cor_semaforo       ; Chama a rotina para enviar a cor correspondente
    
    
    ; Envia o status do Semáforo Esquerdo Jatiúca
    ldi ZL, low(2*msg_esqjati)
    ldi ZH, high(2*msg_esqjati)
    rcall uart_send_string
    lds temp, fase_esqjati
    rcall enviar_cor_semaforo
    
    ; Envia o status do Semáforo Direito Jatiúca
    ldi ZL, low(2*msg_dirjati)
    ldi ZH, high(2*msg_dirjati)
    rcall uart_send_string
    lds temp, fase_dirjati
    rcall enviar_cor_semaforo
    
    ; Envia o status do Semáforo Pajuçara
    ldi ZL, low(2*msg_paju)
    ldi ZH, high(2*msg_paju)
    rcall uart_send_string
    lds temp, fase_paju
    rcall enviar_cor_semaforo
    
    ; Envia uma nova linha (CR + LF) para formatar a saída no terminal
    ldi temp, 13					; 13 é o código ASCII para Carriage Return (CR)
    rcall uart_transmit				
    ldi temp, 10					; 10 é o código ASCII para Line Feed (LF)
    rcall uart_transmit
    
    pop ZH							; Restaura o ponteiro Z
    pop ZL
    ret

; --- Envia a string da cor do semáforo (Vermelho, Amarelo, Verde) baseado no valor em 'temp' ---
enviar_cor_semaforo:
    push ZL
    push ZH
    cpi temp, 0					; Compara a fase com 0 (Vermelho)
    breq enviar_vermelho		
    cpi temp, 1					; Compara a fase com 1 (Amarelo)
    breq enviar_amarelo
    cpi temp, 2					; Compara a fase com 2 (Verde)
    breq enviar_verde
    rjmp fim_enviar_cor			; Se não for nenhum dos valores esperados, apenas retorna

enviar_vermelho:
    ldi ZL, low(2*msg_vermelho)
    ldi ZH, high(2*msg_vermelho)
    rjmp enviar_cor				; Pula para a rotina que envia a string

enviar_amarelo:
    ldi ZL, low(2*msg_amarelo)
    ldi ZH, high(2*msg_amarelo)
    rjmp enviar_cor

enviar_verde:
    ldi ZL, low(2*msg_verde)
    ldi ZH, high(2*msg_verde)

enviar_cor:
    rcall uart_send_string		; Chama a rotina para enviar a string apontada por Z

fim_enviar_cor:
    pop ZH
    pop ZL
    ret

;--------------------------------------------------------------------------------
; DADOS NA MEMÓRIA FLASH (Strings)
;--------------------------------------------------------------------------------
; A diretiva .db (define byte) armazena dados constantes na memória de programa
msg_inicio:     .db "STATUS: ", 0
msg_praia:      .db "Praia-", 0
msg_esqjati:    .db " | EJ-", 0
msg_dirjati:    .db " | DJ-", 0
msg_paju:       .db " | Paju-", 0
msg_vermelho:   .db "Vermelho", 0
msg_amarelo:    .db "Amarelo", 0
msg_verde:      .db "Verde", 0

;--------------------------------------------------------------------------------
; ROTINA DE SERVIÇO DE INTERRUPÇÃO (ISR) DO TIMER1
;--------------------------------------------------------------------------------
; Esta rotina é executada automaticamente uma vez por segundo
timer1_isr:
	; Chama as rotinas para decrementar o tempo de cada semáforo e tratar a mudança de fase
	rcall decrementar_sem_praia
	rcall decrementar_sem_esqjati
	rcall decrementar_sem_dirjati
	rcall decrementar_sem_paju
    
    ; Sinaliza para o loop principal que o status deve ser enviado pela serial
    ldi temp, 1
    sts flag_atualizar_serial, temp
	reti			; Retorno da interrupção. Esta instrução também reabilita as interrupções globais

;--------------------------------------------------------------------------------
; ROTINAS DE LÓGICA DOS SEMÁFOROS (Máquinas de Estado)
;--------------------------------------------------------------------------------
; --- Lógica para o Semáforo da Praia ---
decrementar_sem_praia:
	lds temp, tempo_praia           ; Carrega o tempo atual
    dec temp                        ; Decrementa o tempo
    sts tempo_praia, temp           ; Salva o novo tempo
    brne fim_praia                  ; Se o tempo não chegou a zero, retorna

	; Se o tempo zerou, muda a fase do semáforo
	lds temp, fase_praia            ; Carrega a fase atual
    cpi temp, 0                     ; Se estava no VERMELHO (0)...
    breq fase_verde_praia           ; ...muda para VERDE
    cpi temp, 2                     ; Se estava no VERDE (2)...
    breq fase_amarela_praia         ; ...muda para AMARELO
    cpi temp, 1                     ; Se estava no AMARELO (1)...
    breq reiniciar_semaforos        ; ...muda para VERMELHO e reinicia todo o ciclo

fase_verde_praia:
    ldi temp, 2                     ; Define a nova fase como VERDE (2)
    sts fase_praia, temp
    ldi temp, 25                    ; Carrega o tempo para a fase verde
    sts tempo_praia, temp
    rjmp fim_praia

fase_amarela_praia:
    ldi temp, 1                     ; Define a nova fase como AMARELO (1)
    sts fase_praia, temp
    ldi temp, 4                     ; Carrega o tempo para a fase amarela
    sts tempo_praia, temp
    rjmp fim_praia

reiniciar_semaforos:
; Esta rotina reseta todos os semáforos para seus estados e tempos iniciais,
    ; reiniciando o ciclo de tráfego completo
    ldi temp, 0
    sts fase_praia, temp
    ldi temp, 95
    sts tempo_praia, temp
    ldi temp, 0
    sts fase_esqjati, temp
    ldi temp, 5
    sts tempo_esqjati, temp
    ldi temp, 0
    sts fase_dirjati, temp
    ldi temp, 5
    sts tempo_dirjati, temp
    ldi temp, 0
    sts fase_paju, temp
    ldi temp, 31
    sts tempo_paju, temp

fim_praia:
    ret
; --- Lógica para o Semáforo Esquerda Jatiúca ---
decrementar_sem_esqjati:
	lds temp, tempo_esqjati
	dec temp
	brne fim_esqjati			; Se o tempo não zerou, salva e retorna

	; Se o tempo zerou, muda a fase
	lds temp, fase_esqjati
	cpi temp, 0                     ; Se estava VERMELHO (0)...
    breq fase_vermelho_esqjati      ; ...muda para VERDE
    cpi temp, 2                     ; Se estava VERDE (2)...
    breq fase_amarela_esqjati       ; ...muda para AMARELO.
    cpi temp, 1                     ; Se estava AMARELO (1)...
    breq fase_verde_esqjati         ; ...muda para VERMELHO

fase_vermelho_esqjati:
	ldi temp, 2                     ; Define a nova fase como VERDE (2)
    sts fase_esqjati, temp
    ldi temp, 22                    ; Carrega tempo da fase verde
    sts tempo_esqjati, temp
    rjmp fim_esqjati

fase_amarela_esqjati:
	ldi temp, 1                     ; Define a nova fase como AMARELO (1)
    sts fase_esqjati, temp
    ldi temp, 4                     ; Carrega tempo da fase amarela
    sts tempo_esqjati, temp
    rjmp fim_esqjati

fase_verde_esqjati:
	 ldi temp, 0                     ; Define a nova fase como VERMELHO (0)
    sts fase_esqjati, temp
    ldi temp, 97                    ; Carrega tempo da fase vermelha
    sts tempo_esqjati, temp
    rjmp fim_esqjati

fim_esqjati:
	sts tempo_esqjati, temp			; Salva o tempo (seja o decrementado ou o novo da fase)
	ret

; --- Lógica para o Semáforo Direita Jatiúca ---
; A lógica é similar à do semáforo anterior
decrementar_sem_dirjati:
	lds temp, tempo_dirjati
	dec temp
	brne fim_dirjati

	lds temp, fase_dirjati
	cpi temp, 0
	breq fase_vermelho_dirjati
	cpi temp, 2
	breq fase_amarela_dirjati
	cpi temp, 1
	breq fase_verde_dirjati

fase_vermelho_dirjati:
	ldi temp, 2
	sts fase_dirjati, temp
	ldi temp,86
	sts tempo_dirjati, temp
	rjmp fim_dirjati

fase_amarela_dirjati:
	ldi temp, 1
	sts fase_dirjati, temp
	ldi temp, 4
	sts tempo_dirjati, temp
	rjmp fim_dirjati

fase_verde_dirjati:
	ldi temp, 0
	sts fase_dirjati, temp
	ldi temp, 33
	sts tempo_dirjati, temp
	rjmp fim_dirjati

fim_dirjati:
	sts tempo_dirjati, temp
	ret

; --- Lógica para o Semáforo Pajuçara ---
; A lógica é similar à dos semáforos anteriores
decrementar_sem_paju:
	lds temp, tempo_paju
	dec temp
	brne fim_paju

	lds temp, fase_paju
	cpi temp, 0
	breq fase_vermelho_paju
	cpi temp, 2
	breq fase_amarela_paju
	cpi temp, 1
	breq fase_verde_paju

fase_vermelho_paju:
	ldi temp, 2
	sts fase_paju, temp
	ldi temp, 60
	sts tempo_paju, temp
	rjmp fim_paju

fase_amarela_paju:
	ldi temp, 1
	sts fase_paju, temp
	ldi temp, 4
	sts tempo_paju, temp
	rjmp fim_paju

fase_verde_paju:
	ldi temp, 0
	sts fase_paju, temp
	ldi temp, 55
	sts tempo_paju, temp
	rjmp fim_paju

fim_paju:
	sts tempo_paju, temp
	ret

;--------------------------------------------------------------------------------
; ROTINAS DE CONTROLE DOS DISPLAYS E LEDS
;--------------------------------------------------------------------------------

; --- Rotina principal de Multiplexação ---
; Chama todas as sub-rotinas que atualizam os componentes visuais
multiplex:
	rcall mostrar_display
	rcall semaforo_praia
	rcall semaforo_esqjati
	rcall semaforo_dirjati
	rcall semaforo_paju
	ret

; --- Atualiza o display de 7 segmentos ---
; Mostra o tempo do semáforo da Praia em dois displays de 7 segmentos
mostrar_display:
	lds temp, tempo_praia           ; Carrega o tempo do semáforo da Praia
    clr tens                        ; Zera o registrador da dezena

; Loop para separar o número em dezenas e unidades
div_loop:
	cpi temp, 10                    ; Compara o tempo com 10
    brlo fim_div                    ; Se for menor, o que sobrou é a unidade
    subi temp, 10                   ; Subtrai 10 do tempo
    inc tens                        ; Incrementa o contador de dezenas
    rjmp div_loop                   ; Repete o processo

fim_div:
	mov count, temp					; O resto da divisão é a unidade

	 ; --- Lógica de Multiplexação ---
    ; Acende um display de cada vez, muito rapidamente, para criar a ilusão
    ; de que ambos estão acesos ao mesmo tempo (Persistência da Visão)

	; Mostra o dígito da UNIDADE
	in temp_reg, PORTC              ; Lê o estado atual do PORTC
    andi temp_reg, 0b11110000       ; Zera os 4 bits inferiores (PC0-PC3), preservando os superiores
    or temp_reg, count              ; Combina com o valor da unidade
    out PORTC, temp_reg             ; Envia o valor BCD da unidade para o display
    cbi PORTB, PB0                  ; Desliga o display da dezena (GND no transistor)
    sbi PORTB, PB1                  ; Liga o display da unidade (GND no transistor)
    rcall delay5ms                  ; Pequeno atraso para o dígito ser visível


	; Mostra o dígito da DEZENA
	in temp_reg, PORTC              ; Lê o estado atual do PORTC novamente
    andi temp_reg, 0b11110000       ; Zera os 4 bits inferiores
    or temp_reg, tens               ; Combina com o valor da dezena
    out PORTC, temp_reg             ; Envia o valor BCD da dezena para o display
    cbi PORTB, PB1                  ; Desliga o display da unidade
    sbi PORTB, PB0                  ; Liga o display da dezena
    rcall delay5ms                  ; Pequeno atraso
    ret

; --- Acende os LEDs do Semáforo da Praia ---
semaforo_praia:
    lds temp, fase_praia			; Carrega a fase atual
	cpi temp, 0
	breq luz_vermelho_praia
	cpi temp, 1
	breq luz_amarela_praia
	cpi temp, 2
	breq luz_verde_praia
	ret

luz_vermelho_praia:
    sbi PORTB, PB2                  ; Acende LED Vermelho (PB2)
    cbi PORTB, PB3                  ; Apaga LED Amarelo (PB3)
    cbi PORTB, PB4                  ; Apaga LED Verde (PB4)
    ret

luz_amarela_praia:
    cbi PORTB, PB2
    sbi PORTB, PB3					; Acende LED Amarelo
    cbi PORTB, PB4
    ret

luz_verde_praia:
    cbi PORTB, PB2
    cbi PORTB, PB3
    sbi PORTB, PB4					; Acende LED Verde
    ret


; --- Acende os LEDs do Semáforo Esquerda Jatiúca ---
semaforo_esqjati:
    lds temp, fase_esqjati
	cpi temp, 0
	breq luz_vermelho_esqjati
	cpi temp, 1
	breq luz_amarela_esqjati
	cpi temp, 2
	breq luz_verde_esqjati
	ret

luz_vermelho_esqjati:
    sbi PORTD, PD5					; Acende LED Vermelho (PD5)
    cbi PORTD, PD6
    cbi PORTD, PD7
    ret

luz_amarela_esqjati:
    cbi PORTD, PD5
    sbi PORTD, PD6					; Acende LED Amarelo (PD6)
    cbi PORTD, PD7
    ret

luz_verde_esqjati:
    cbi PORTD, PD5
    cbi PORTD, PD6
    sbi PORTD, PD7					; Acende LED Verde (PD7)
    ret


; --- Acende os LEDs do Semáforo Direita Jatiúca ---
semaforo_dirjati:
    lds temp, fase_dirjati
	cpi temp, 0
	breq luz_vermelho_dirjati
	cpi temp, 1
	breq luz_amarela_dirjati
	cpi temp, 2
	breq luz_verde_dirjati
	ret

luz_vermelho_dirjati:
    sbi PORTD, PD2					 ; Acende LED Vermelho (PD2)
    cbi PORTD, PD3
    cbi PORTD, PD4
    ret

luz_amarela_dirjati:
    cbi PORTD, PD2
    sbi PORTD, PD3					; Acende LED Amarelo (PD3)
    cbi PORTD, PD4
    ret

luz_verde_dirjati:
    cbi PORTD, PD2
    cbi PORTD, PD3
    sbi PORTD, PD4					; Acende LED Verde (PD4)
    ret

semaforo_paju:
    lds temp, fase_paju
	cpi temp, 0
	breq luz_vermelho_paju
	cpi temp, 1
	breq luz_amarela_paju
	cpi temp, 2
	breq luz_verde_paju
	ret

luz_vermelho_paju:
    sbi PORTB, PB5					; Acende LED Vermelho (PB5)
    cbi PORTC, PC4
    cbi PORTC, PC5
    ret

luz_amarela_paju:
    cbi PORTB, PB5
    sbi PORTC, PC4					; Acende LED Amarelo (PC4).
    cbi PORTC, PC5
    ret

luz_verde_paju:
    cbi PORTB, PB5
    cbi PORTC, PC4
    sbi PORTC, PC5					 ; Acende LED Verde (PC5)
    ret


;--------------------------------------------------------------------------------
; ROTINA DE DELAY (ATRASO)
;--------------------------------------------------------------------------------
; Cria um atraso de aproximadamente 5 milissegundos (software delay)
delay5ms:
    push r19                        ; Salva registradores que serão usados
    push r20
    ldi r19, 50                     ; Carrega o contador do loop externo
d5_loop1:
    ldi r20, 200                    ; Carrega o contador do loop interno
d5_loop2:
    dec r20                         ; Decrementa o contador interno
    brne d5_loop2                   ; Repete até r20 ser zero
    dec r19                         ; Decrementa o contador externo
    brne d5_loop1                   ; Repete até r19 ser zero
    pop r20                         ; Restaura os registradores
    pop r19
    ret