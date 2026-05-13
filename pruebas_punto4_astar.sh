#!/bin/bash

OUT="resultados_punto4_astar.txt"
CLEAN="metricas_punto4_astar.txt"

rm -f "$OUT" "$CLEAN"

echo "============================================================" | tee -a "$OUT"
echo "PUNTO 4 - A* Y HEURISTICAS" | tee -a "$OUT"
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
        echo "OK | $NAME | ${DURATION}s" >> "$CLEAN"
    else
        echo "ERROR | $NAME | ${DURATION}s" >> "$CLEAN"
    fi
}

echo "RESUMEN PUNTO 4" >> "$CLEAN"
echo "Fecha: $(date)" >> "$CLEAN"
echo "============================================================" >> "$CLEAN"

SIMPLE_LAYOUTS=(
    "tinyBase"
    "smallRescue"
    "openRescue"
    "cornerRescue"
    "narrowRescue"
    "mediumRescue"
    "warehouseRescue"
)

MULTI_LAYOUTS=(
    "tinyMulti"
    "smallMulti"
    "duoRescue"
    "crossMulti"
)

HEURISTICS=(
    "nullHeuristic"
    "ignorePreconditionsHeuristic"
    "ignoreDeleteListsHeuristic"
)

echo "" | tee -a "$OUT"
echo "==================== CONTROL BASE ====================" | tee -a "$OUT"

run_test "CONTROL - tinyBaseSearch - tinyBase" \
"python3 main.py -p SimpleRescueProblem -f tinyBaseSearch -l tinyBase -t"

echo "" | tee -a "$OUT"
echo "==================== FORWARD BFS SIN HEURISTICA ====================" | tee -a "$OUT"

for LAYOUT in "${SIMPLE_LAYOUTS[@]}"; do
    run_test "PUNTO 4 CONTROL - forwardBFS - Layout simple/$LAYOUT" \
    "python3 main.py -p SimpleRescueProblem -f forwardBFS -l $LAYOUT -t"
done

echo "" | tee -a "$OUT"
echo "==================== A* EN LAYOUTS SIMPLE ====================" | tee -a "$OUT"

for LAYOUT in "${SIMPLE_LAYOUTS[@]}"; do
    for H in "${HEURISTICS[@]}"; do
        run_test "PUNTO 4 - A* - Layout simple/$LAYOUT - Heurística $H" \
        "python3 main.py -p SimpleRescueProblem -f aStarPlanner -h $H -l $LAYOUT -t"
    done
done

echo "" | tee -a "$OUT"
echo "==================== A* EXTRA EN MULTIRESCUE ====================" | tee -a "$OUT"

for LAYOUT in "${MULTI_LAYOUTS[@]}"; do
    for H in "${HEURISTICS[@]}"; do
        run_test "PUNTO 4 EXTRA - A* MultiRescue - Layout multi/$LAYOUT - Heurística $H" \
        "python3 main.py -p MultiRescueProblem -f aStarPlanner -h $H -l $LAYOUT -t"
    done
done

echo "" | tee -a "$OUT"
echo "============================================================" | tee -a "$OUT"
echo "FIN PUNTO 4" | tee -a "$OUT"
echo "Archivo completo: $OUT" | tee -a "$OUT"
echo "Resumen: $CLEAN" | tee -a "$OUT"
echo "============================================================" | tee -a "$OUT"

echo ""
echo "Listo Punto 4."
echo "Resultados completos: $OUT"
echo "Resumen: $CLEAN"
