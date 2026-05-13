#!/bin/bash

OUT="resultados_punto5_htn.txt"
SUMMARY="metricas_punto5_htn.txt"
CLEAN="limpio_punto5_htn.txt"

rm -f "$OUT" "$SUMMARY" "$CLEAN"

echo "============================================================" | tee -a "$OUT"
echo "PUNTO 5 - PLANIFICACION JERARQUICA HTN" | tee -a "$OUT"
echo "Fecha: $(date)" | tee -a "$OUT"
echo "Carpeta: $(pwd)" | tee -a "$OUT"
echo "============================================================" | tee -a "$OUT"

run_test () {
    NAME="$1"
    CMD="$2"

    echo "" | tee -a "$OUT"
    echo "============================================================" | tee -a "$OUT"
    echo "$NAME" | tee -a "$OUT"
    echo "Comando: $CMD" | tee -a "$OUT"
    echo "============================================================" | tee -a "$OUT"

    START=$(date +%s)
    eval "$CMD" 2>&1 | tee -a "$OUT"
    STATUS=${PIPESTATUS[0]}
    END=$(date +%s)
    DURATION=$((END - START))

    echo "" | tee -a "$OUT"
    echo "Estado del comando: $STATUS" | tee -a "$OUT"
    echo "Duración aproximada: ${DURATION}s" | tee -a "$OUT"

    if [ $STATUS -eq 0 ]; then
        echo "OK | $NAME | ${DURATION}s" >> "$SUMMARY"
    else
        echo "ERROR | $NAME | ${DURATION}s" >> "$SUMMARY"
    fi
}

echo "RESUMEN PUNTO 5 - HTN" >> "$SUMMARY"
echo "Fecha: $(date)" >> "$SUMMARY"
echo "============================================================" >> "$SUMMARY"

echo ""
echo "Iniciando pruebas del Punto 5..."
echo "Archivo completo: $OUT"
echo "Resumen: $SUMMARY"
echo ""

# ============================================================
# PUNTO 5A - HTN EN layouts/htn/
# Requisito del enunciado: probar layouts/htn/ con flag -m
# ============================================================

run_test "PUNTO 5A - HTN - SimpleRescueProblem - tinyHTN" \
"python3 main.py -p SimpleRescueProblem -f hierarchicalSearch -l tinyHTN -m -t"

run_test "PUNTO 5A - HTN - SimpleRescueProblem - htnBase" \
"python3 main.py -p SimpleRescueProblem -f hierarchicalSearch -l htnBase -m -t"

# ============================================================
# PUNTO 5A EXTRA - HTN EN layouts/simple/
# Sirve para probar que la jerarquía también funciona
# fuera de los layouts htn.
# ============================================================

run_test "PUNTO 5A EXTRA - HTN - SimpleRescueProblem - tinyBase" \
"python3 main.py -p SimpleRescueProblem -f hierarchicalSearch -l tinyBase -m -t"

run_test "PUNTO 5A EXTRA - HTN - SimpleRescueProblem - smallRescue" \
"python3 main.py -p SimpleRescueProblem -f hierarchicalSearch -l smallRescue -m -t"

run_test "PUNTO 5A EXTRA - HTN - SimpleRescueProblem - openRescue" \
"python3 main.py -p SimpleRescueProblem -f hierarchicalSearch -l openRescue -m -t"

run_test "PUNTO 5A EXTRA - HTN - SimpleRescueProblem - cornerRescue" \
"python3 main.py -p SimpleRescueProblem -f hierarchicalSearch -l cornerRescue -m -t"

run_test "PUNTO 5A EXTRA - HTN - SimpleRescueProblem - mediumRescue" \
"python3 main.py -p SimpleRescueProblem -f hierarchicalSearch -l mediumRescue -m -t"

run_test "PUNTO 5A EXTRA - HTN - SimpleRescueProblem - warehouseRescue" \
"python3 main.py -p SimpleRescueProblem -f hierarchicalSearch -l warehouseRescue -m -t"

# ============================================================
# PUNTO 5B - HTN EN layouts/multi/
# Requisito del enunciado: probar MultiRescueProblem.
# ============================================================

run_test "PUNTO 5B - HTN - MultiRescueProblem - tinyMulti" \
"python3 main.py -p MultiRescueProblem -f hierarchicalSearch -l tinyMulti -m -t"

run_test "PUNTO 5B - HTN - MultiRescueProblem - smallMulti" \
"python3 main.py -p MultiRescueProblem -f hierarchicalSearch -l smallMulti -m -t"

run_test "PUNTO 5B - HTN - MultiRescueProblem - duoRescue" \
"python3 main.py -p MultiRescueProblem -f hierarchicalSearch -l duoRescue -m -t"

run_test "PUNTO 5B - HTN - MultiRescueProblem - crossMulti" \
"python3 main.py -p MultiRescueProblem -f hierarchicalSearch -l crossMulti -m -t"

echo "" | tee -a "$OUT"
echo "============================================================" | tee -a "$OUT"
echo "FIN PRUEBAS PUNTO 5" | tee -a "$OUT"
echo "Archivo completo: $OUT" | tee -a "$OUT"
echo "Resumen: $SUMMARY" | tee -a "$OUT"
echo "============================================================" | tee -a "$OUT"

grep -E "PUNTO 5|Problema:|Layout:|Planificador:|Tiempo de planificación|Estados expandidos|Longitud del plan|Misión completada|No se encontró|Traceback|IndexError|Estado del comando|Duración aproximada" "$OUT" > "$CLEAN"

echo ""
echo "Listo Punto 5."
echo "Resultados completos: $OUT"
echo "Resumen: $SUMMARY"
echo "Archivo limpio: $CLEAN"
