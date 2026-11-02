# Thesis build. Needs pdflatex, biber, makeglossaries (all in TeX Live).
# Optional: texlive-science (siunitx; a fallback is built in), latexmk.
MAIN  := main
LATEX := pdflatex -interaction=nonstopmode -halt-on-error -file-line-error

.PHONY: all draft quick clean distclean watch

all: $(MAIN).pdf

$(MAIN).pdf: FORCE
	$(LATEX) $(MAIN)
	biber $(MAIN)
	makeglossaries $(MAIN)
	$(LATEX) $(MAIN)
	$(LATEX) $(MAIN)

# TODO notes + line numbers visible
draft: FORCE
	$(LATEX) "\def\thesisdraft{}\input{$(MAIN)}"
	biber $(MAIN)
	makeglossaries $(MAIN)
	$(LATEX) "\def\thesisdraft{}\input{$(MAIN)}"
	$(LATEX) "\def\thesisdraft{}\input{$(MAIN)}"

# one pass, no bibliography/glossary refresh
quick: FORCE
	$(LATEX) $(MAIN)

# rebuild on save (needs latexmk)
watch:
	latexmk -pdf -pvc $(MAIN)

clean:
	rm -f *.aux chapters/*.aux appendices/*.aux frontmatter/*.aux *.bbl *.bcf *.blg \
	      *.run.xml *.log *.out *.toc *.lof *.lot *.glo *.gls *.glg *.acn *.acr *.alg \
	      *.ist *.slo *.sls *.slg *.glsdefs *.tdo *.synctex.gz *.fls *.fdb_latexmk

distclean: clean
	rm -f $(MAIN).pdf

FORCE:
