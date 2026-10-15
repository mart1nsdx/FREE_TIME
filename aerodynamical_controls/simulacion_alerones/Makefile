# ═══════════════════════════════════════════════════════════════════════════
#  simulacion_alerones — build (macOS: GLUT/OpenGL de los frameworks del SDK,
#  mismo stack que unmanned/drone_model.cpp; en Linux usa GL/glut.h + -lglut).
#
#  'make help' lista los targets.
# ═══════════════════════════════════════════════════════════════════════════
CXX      := clang++
CXXFLAGS := -std=c++17 -O2 -Wall -DGL_SILENCE_DEPRECATION

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  GLLIBS := -framework OpenGL -framework GLUT
else
  GLLIBS := -lglut -lGLU -lGL -lm
endif

OBJ    := fisica_alerones.o alerones_graficos.o main.o
PY     := python3 alerones_py/main.py
SCRIPT := desacoplo amortiguamiento aeroelasticidad actuador autopiloto regresion

.PHONY: all run headless test python python-test paridad scripts figuras clean help

all: alerones

alerones: $(OBJ)              ## enlaza el simulador grafico
	$(CXX) $(CXXFLAGS) $(OBJ) -o $@ $(GLLIBS)

run: alerones                 ## compila y ejecuta la simulacion grafica
	./alerones

headless: alerones            ## REPL por stdin (scripting / regresion)
	./alerones --headless

test: run_tests               ## casos de validacion (§7), sin grafica
	./run_tests

run_tests: tests.cpp fisica_alerones.o fisica_alerones.hpp
	$(CXX) $(CXXFLAGS) tests.cpp fisica_alerones.o -o $@

python:                       ## prototipo de referencia en Python (§8)
	$(PY)

python-test:                  ## los mismos casos de §7 en Python
	$(PY) --test

python-repl:                  ## REPL de Python, equivalente a 'make headless'
	$(PY) --repl

scripts: alerones             ## pasa scripts/*.txt por el modo headless
	@for s in $(SCRIPT); do \
	  echo "═══ scripts/$$s.txt ═══"; \
	  ./alerones --headless < scripts/$$s.txt; \
	done

# Los dos modelos deben dar EXACTAMENTE la misma traza, byte a byte.  Es la
# prueba mas fuerte que tiene el modulo: dos implementaciones independientes
# que no se pueden copiar un error numerico la una a la otra.
paridad: alerones             ## C++ vs Python: tests y scripts, byte a byte
	@ok=1; \
	./run_tests > /tmp/alr_c.txt 2>&1 || true; \
	$(PY) --test > /tmp/alr_p.txt 2>&1 || true; \
	if diff -q /tmp/alr_c.txt /tmp/alr_p.txt >/dev/null; \
	  then echo "  [ ok ] tests (§7)"; \
	  else echo "  [DIFF] tests (§7)"; diff /tmp/alr_c.txt /tmp/alr_p.txt | head -20; ok=0; fi; \
	for s in $(SCRIPT); do \
	  ./alerones --headless      < scripts/$$s.txt > /tmp/alr_c.txt 2>&1; \
	  $(PY) --repl               < scripts/$$s.txt > /tmp/alr_p.txt 2>&1; \
	  if diff -q /tmp/alr_c.txt /tmp/alr_p.txt >/dev/null; \
	    then echo "  [ ok ] scripts/$$s.txt"; \
	    else echo "  [DIFF] scripts/$$s.txt"; diff /tmp/alr_c.txt /tmp/alr_p.txt | head -20; ok=0; fi; \
	done; \
	rm -f /tmp/alr_c.txt /tmp/alr_p.txt; \
	test $$ok -eq 1 && echo "  C++ y Python coinciden byte a byte."

# CSV para las figuras del libro.  Columnas:
#   t, phi, theta, psi, p, q, r, d1..d4, dc1..dc4, L, M, N
figuras: alerones             ## genera figuras/*.csv (escalon, amortiguado, AP)
	@mkdir -p figuras
	@printf 'servo off\ndamp off\npitch 3\ncsv 20 0.05 figuras/sin_amortiguar.csv\n' \
	  | ./alerones --headless > /dev/null
	@printf 'servo off\ndamp on\npitch 3\ncsv 200 0.5 figuras/amortiguado.csv\n' \
	  | ./alerones --headless > /dev/null
	@printf 'ap on\nref 10 5 -8\ncsv 30 0.05 figuras/autopiloto.csv\n' \
	  | ./alerones --headless > /dev/null
	@printf 'servo on\nset tau_servo 0.05\nset rate_limit 100\npitch 10\ncsv 0.5 0.005 figuras/actuador.csv\n' \
	  | ./alerones --headless > /dev/null
	@printf 'damp off\nset K_theta 5\npitch 10\ncsv 1 0.002 figuras/aeroelastico.csv\n' \
	  | ./alerones --headless > /dev/null
	@ls -1 figuras/

clean:                        ## borra objetos, binarios, CSV y __pycache__
	rm -f *.o alerones run_tests
	rm -rf figuras alerones_py/__pycache__

help:                         ## esta lista
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
	 awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}'

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

fisica_alerones.o:    fisica_alerones.cpp fisica_alerones.hpp
alerones_graficos.o:  alerones_graficos.cpp alerones_graficos.hpp fisica_alerones.hpp
main.o:               main.cpp alerones_graficos.hpp fisica_alerones.hpp
