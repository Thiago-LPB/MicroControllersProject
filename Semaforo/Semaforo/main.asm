.def temp  = r16
.def count = r17			; unidade
.def tens  = r18			; dezena

.cseg
.org 0x0000
    rjmp reset
.org 0x0016
    rjmp timer1_isr    ; vetor Timer1 Compare A

;----------------------------------------
; Memória SRAM para semáforos
;----------------------------------------
.dseg
fase_praia: .byte 1
tempo_praia: .byte 1

fase_esqjati: .byte 1
tempo_esqjati: .byte 1

fase_dirjati: .byte 1
tempo_dirjati: .byte 1

fase_paju: .byte 1
tempo_paju: .byte 1

flag_atualizar_serial: .byte 1  ; flag para enviar status

.cseg
;----------------------------------------
reset:
    ; Inicializa stack
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp

    ; Configura UART (9600 baud @ 16MHz)
    rcall uart_init

    ; Configura PORTC C0–C3 como saída (BCD)
    ldi temp, 0x0F
    out DDRC, temp

    ; Configura PORTB0 e PORTB1 como saída (TIP122)
    ldi temp, (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB5)|(1<<PB6)|(1<<PB7)
    out DDRB, temp

	ldi temp, (1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7)
	out DDRD, temp

	; Inicia semáforos todos no vermelho
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

    ldi temp, 0
    sts flag_atualizar_serial, temp

    ; Configura Timer1 para 1 Hz (CTC)
    ldi temp, (1<<WGM12)
    sts TCCR1B, temp

    ldi temp, high(15624)
    sts OCR1AH, temp
    ldi temp, low(15624)
    sts OCR1AL, temp

    ldi temp, (1<<CS12)|(1<<CS10)|(1<<WGM12)
    sts TCCR1B, temp

    ldi temp, (1<<OCIE1A)
    sts TIMSK1, temp

    sei

main_lp:
    ; Verifica se precisa enviar status
    lds temp, flag_atualizar_serial
    cpi temp, 1
    brne continua_main
    rcall enviar_status_serial
    ldi temp, 0
    sts flag_atualizar_serial, temp

continua_main:
    rcall multiplex
    rjmp main_lp

;----------------------------------------
; Inicializa UART - 9600 baud @ 16MHz
;----------------------------------------
uart_init:
    ; UBRR = 103 para 9600 baud @ 16MHz
    ldi temp, 0
    sts UBRR0H, temp
    ldi temp, 103
    sts UBRR0L, temp
    
    ; Habilita transmissão
    ldi temp, (1<<TXEN0)
    sts UCSR0B, temp
    
    ; 8 bits, 1 stop bit, sem paridade
    ldi temp, (1<<UCSZ01)|(1<<UCSZ00)
    sts UCSR0C, temp
    ret

;----------------------------------------
; Transmite um byte via UART
;----------------------------------------
uart_transmit:
    push r19
uart_wait:
    lds r19, UCSR0A
    sbrs r19, UDRE0
    rjmp uart_wait
    sts UDR0, temp
    pop r19
    ret

;----------------------------------------
; Envia string via UART (Z aponta para string na flash)
;----------------------------------------
uart_send_string:
    push r19
    push ZL
    push ZH
uart_str_loop:
    lpm r19, Z+
    cpi r19, 0
    breq uart_str_end
    mov temp, r19
    rcall uart_transmit
    rjmp uart_str_loop
uart_str_end:
    pop ZH
    pop ZL
    pop r19
    ret

;----------------------------------------
; Envia status de todos os semáforos
;----------------------------------------
enviar_status_serial:
    push ZL
    push ZH
    
    ; Cabeçalho
    ldi ZL, low(2*msg_inicio)
    ldi ZH, high(2*msg_inicio)
    rcall uart_send_string
    
    ; Semáforo Praia
    ldi ZL, low(2*msg_praia)
    ldi ZH, high(2*msg_praia)
    rcall uart_send_string
    lds temp, fase_praia
    rcall enviar_cor_semaforo
    
    ; Semáforo Esquerdo Jatiuca
    ldi ZL, low(2*msg_esqjati)
    ldi ZH, high(2*msg_esqjati)
    rcall uart_send_string
    lds temp, fase_esqjati
    rcall enviar_cor_semaforo
    
    ; Semáforo Direito Jatiuca
    ldi ZL, low(2*msg_dirjati)
    ldi ZH, high(2*msg_dirjati)
    rcall uart_send_string
    lds temp, fase_dirjati
    rcall enviar_cor_semaforo
    
    ; Semáforo Pajuçara
    ldi ZL, low(2*msg_paju)
    ldi ZH, high(2*msg_paju)
    rcall uart_send_string
    lds temp, fase_paju
    rcall enviar_cor_semaforo
    
    ; Nova linha
    ldi temp, 13
    rcall uart_transmit
    ldi temp, 10
    rcall uart_transmit
    
    pop ZH
    pop ZL
    ret

;----------------------------------------
; Envia cor do semáforo baseado na fase
;----------------------------------------
enviar_cor_semaforo:
    push ZL
    push ZH
    cpi temp, 0
    breq enviar_vermelho
    cpi temp, 1
    breq enviar_amarelo
    cpi temp, 2
    breq enviar_verde
    rjmp fim_enviar_cor

enviar_vermelho:
    ldi ZL, low(2*msg_vermelho)
    ldi ZH, high(2*msg_vermelho)
    rjmp enviar_cor

enviar_amarelo:
    ldi ZL, low(2*msg_amarelo)
    ldi ZH, high(2*msg_amarelo)
    rjmp enviar_cor

enviar_verde:
    ldi ZL, low(2*msg_verde)
    ldi ZH, high(2*msg_verde)

enviar_cor:
    rcall uart_send_string

fim_enviar_cor:
    pop ZH
    pop ZL
    ret

;----------------------------------------
; Strings na memória flash
;----------------------------------------
msg_inicio:     .db "STATUS: ", 0
msg_praia:      .db "Praia-", 0
msg_esqjati:    .db " | EJ-", 0
msg_dirjati:    .db " | DJ-", 0
msg_paju:       .db " | Paju-", 0
msg_vermelho:   .db "Vermelho", 0
msg_amarelo:    .db "Amarelo", 0
msg_verde:      .db "Verde", 0

;----------------------------------------
; ISR Timer1 - decrementar tempos
;----------------------------------------
timer1_isr:
	rcall decrementar_sem_praia
	rcall decrementar_sem_esqjati
	rcall decrementar_sem_dirjati
	rcall decrementar_sem_paju
    
    ; Seta flag para atualizar serial
    ldi temp, 1
    sts flag_atualizar_serial, temp
	reti

;----------------------------------------
; Rotinas de decremento (mantidas iguais)
;----------------------------------------
decrementar_sem_praia:
	lds temp, tempo_praia
	dec temp
	sts tempo_praia, temp
	brne fim_praia

	lds temp, fase_praia
	cpi temp, 0
	breq fase_verde_praia
	cpi temp, 2
	breq fase_amarela_praia
	cpi temp, 1
	breq reiniciar_semaforos

fase_verde_praia:
    ldi temp, 2
    sts fase_praia, temp
    ldi temp, 25
    sts tempo_praia, temp
    rjmp fim_praia

fase_amarela_praia:
    ldi temp, 1
    sts fase_praia, temp
    ldi temp, 4
    sts tempo_praia, temp
    rjmp fim_praia

reiniciar_semaforos:
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

decrementar_sem_esqjati:
	lds temp, tempo_esqjati
	dec temp
	brne fim_esqjati

	lds temp, fase_esqjati
	cpi temp, 0
	breq fase_vermelho_esqjati
	cpi temp, 2
	breq fase_amarela_esqjati
	cpi temp, 1
	breq fase_verde_esqjati

fase_vermelho_esqjati:
	ldi temp, 2
	sts fase_esqjati, temp
	ldi temp, 22
	sts tempo_esqjati, temp
	rjmp fim_esqjati

fase_amarela_esqjati:
	ldi temp, 1
	sts fase_esqjati, temp
	ldi temp, 4
	sts tempo_esqjati, temp
	rjmp fim_esqjati

fase_verde_esqjati:
	ldi temp, 0
	sts fase_esqjati, temp
	ldi temp, 97
	sts tempo_esqjati, temp
	rjmp fim_esqjati

fim_esqjati:
	sts tempo_esqjati, temp
	ret

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

;----------------------------------------
; Multiplexação (mantida igual)
;----------------------------------------
multiplex:
	rcall mostrar_display
	rcall semaforo_praia
	rcall semaforo_esqjati
	rcall semaforo_dirjati
	rcall semaforo_paju
	ret

mostrar_display:
	lds temp, tempo_praia
	clr tens
div_loop:
	cpi temp,10
	brlo fim_div
	subi temp, 10
	inc tens
	rjmp div_loop

fim_div:
	mov count, temp

    cbi PORTB, PB0
    sbi PORTB, PB1
    mov temp, count
    out PORTC, temp
    rcall delay5ms

    cbi PORTB, PB1
    sbi PORTB, PB0
    mov temp, tens
    out PORTC, temp
    rcall delay5ms
	ret

semaforo_praia:
    lds temp, fase_praia
	cpi temp, 0
	breq luz_vermelho_praia
	cpi temp, 1
	breq luz_amarela_praia
	cpi temp, 2
	breq luz_verde_praia
	ret

luz_vermelho_praia:
    sbi PORTB, PB2
    cbi PORTB, PB3
    cbi PORTB, PB4
    ret

luz_amarela_praia:
    cbi PORTB, PB2
    sbi PORTB, PB3
    cbi PORTB, PB4
    ret

luz_verde_praia:
    cbi PORTB, PB2
    cbi PORTB, PB3
    sbi PORTB, PB4
    ret

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
    sbi PORTD, PD5
    cbi PORTD, PD6
    cbi PORTD, PD7
    ret

luz_amarela_esqjati:
    cbi PORTD, PD5
    sbi PORTD, PD6
    cbi PORTD, PD7
    ret

luz_verde_esqjati:
    cbi PORTD, PD5
    cbi PORTD, PD6
    sbi PORTD, PD7
    ret

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
    sbi PORTD, PD2
    cbi PORTD, PD3
    cbi PORTD, PD4
    ret

luz_amarela_dirjati:
    cbi PORTD, PD2
    sbi PORTD, PD3
    cbi PORTD, PD4
    ret

luz_verde_dirjati:
    cbi PORTD, PD2
    cbi PORTD, PD3
    sbi PORTD, PD4
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
    sbi PORTB, PB5
    cbi PORTB, PB6
    cbi PORTB, PB7
    ret

luz_amarela_paju:
    cbi PORTB, PB5
    sbi PORTB, PB6
    cbi PORTB, PB7
    ret

luz_verde_paju:
    cbi PORTB, PB5
    cbi PORTB, PB6
    sbi PORTB, PB7
    ret

delay5ms:
	push r19
	push r20
    ldi r19, 50
d5_loop1:
    ldi r20, 200
d5_loop2:
    dec r20
    brne d5_loop2
    dec r19
    brne d5_loop1
	pop r20
	pop r19
    ret