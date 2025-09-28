.def temp  = r16
.def count = r17   ; unidade
.def tens  = r18   ; dezena

.cseg
.org 0x0000
    rjmp reset
.org 0x0016
    rjmp timer1_isr    ; vetor Timer1 Compare A

;----------------------------------------
reset:
    ; Inicializa stack
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp

    ; Configura PORTC C0–C3 como saída (BCD)
    ldi temp, 0x0F
    out DDRC, temp

    ; Configura PORTB0 e PORTB1 como saída (TIP122)
    ldi temp, (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4)
    out DDRB, temp

    ; Inicializa contador: 99
    ldi count, 9
    ldi tens, 9

    ; Configura Timer1 para 1 Hz (CTC)
    ldi temp, (1<<WGM12)
    sts TCCR1B, temp

    ldi temp, high(15624)   ; para 1 Hz @ 16MHz / 1024
    sts OCR1AH, temp
    ldi temp, low(15624)
    sts OCR1AL, temp

    ; Prescaler 1024
    ldi temp, (1<<CS12)|(1<<CS10)|(1<<WGM12)
    sts TCCR1B, temp

    ; Habilita interrupção Compare A
    ldi temp, (1<<OCIE1A)
    sts TIMSK1, temp

    sei     ; habilita interrupções globais

main_lp:
    rjmp multiplex   ; loop principal chama multiplexação

;----------------------------------------
; ISR Timer1 Compare A - decrementa contador
;----------------------------------------
timer1_isr:
    dec count
    brge continue_timer   ; se >=0, continua
    ldi count, 9          ; unidade reinicia
    dec tens
    brge continue_timer
    ldi tens, 9           ; dezena reinicia se passar de 0
continue_timer:
    reti

;----------------------------------------
; Multiplexação dos displays
;----------------------------------------
multiplex:
	rcall semaforo
    ; Exibe unidade
    cbi PORTB, PB0       ; desliga dezena
    sbi PORTB, PB1       ; liga unidade
    mov temp, count
    out PORTC, temp
    rcall delay5ms

    ; Exibe dezena
    cbi PORTB, PB1       ; desliga unidade
    sbi PORTB, PB0       ; liga dezena
    mov temp, tens
    out PORTC, temp
    rcall delay5ms

    rjmp multiplex

;----------------------------------------
; Controle do semáforo
;----------------------------------------
semaforo:
    ; Intervalos baseados na contagem total (tens:count)
    ; Aqui usei só a dezena para simplificar
    mov temp, tens        

    ; Vermelho: quando dezena >= 7
    cpi temp, 7
    brge luz_vermelha

    ; Amarelo: quando dezena entre 5 e 6
    cpi temp, 5
    brge luz_amarela

    ; Verde: quando dezena < 5
    rjmp luz_verde

luz_vermelha:
    sbi PORTB, PB2   ; acende vermelho
    cbi PORTB, PB3   ; apaga amarelo
    cbi PORTB, PB4   ; apaga verde
    rjmp fim_semaforo

luz_amarela:
    cbi PORTB, PB2
    sbi PORTB, PB3
    cbi PORTB, PB4
    rjmp fim_semaforo

luz_verde:
    cbi PORTB, PB2
    cbi PORTB, PB3
    sbi PORTB, PB4
    rjmp fim_semaforo

fim_semaforo:
    ret

;----------------------------------------
; Delay ~5 ms (ajustar conforme clock)
;----------------------------------------
delay5ms:
    ldi r20, 50
d5_loop1:
    ldi r21, 200
d5_loop2:
    dec r21
    brne d5_loop2
    dec r20
    brne d5_loop1
    ret
