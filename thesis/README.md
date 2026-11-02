# Thesis

- `latex/`: the thesis document (UNSW report format). Build with `make` (or `make draft` to show TODO notes and
  line numbers). Needs pdflatex, biber and makeglossaries. `siunitx` (package texlive-science) is optional; there
  is a fallback. `latexmk` (for `make watch`) is optional. The `latexmkrc` also makes it build on Overleaf.
  - `main.tex`: document order; `preamble/metadata.tex`: title, zID, degree, supervisor, dates.
  - `chapters/`, `appendices/`, `frontmatter/`: headings, labels and planning comments only, no prose yet.
  - `preamble/macros.tex`: notation and helpers (`\TODO`, `\FIG`, `\CITE`, `\CHECK`, `\placeholderfig`).
  - `preamble/glossary.tex`: abbreviations and symbols (printed once used with `\gls{...}`).
  - `references.bib`: copied from the Thesis A literature review.
  - `figures/<chapter>/`, `data/` (CSV for pgfplots), `tables/`, `code/`.
  - `snippets.tex`: copy-paste templates for figures, tables, plots and equations (not compiled).
- `planning/`: what to write and why. Start with `planning/README.md`.
