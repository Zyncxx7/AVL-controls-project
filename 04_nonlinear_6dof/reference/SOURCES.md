# Model sources

- `aerodynamics.pdf`: C172P aerodynamic reference document. The
  aerodynamic geometry, reference point (page 2), tables and derivatives come
  from this document.
- `c172p.xml`: retrieved 2026-11-02 from
  https://raw.githubusercontent.com/JSBSim-Team/jsbsim/master/aircraft/c172p/c172p.xml
  SHA-256: `0af58c613f7bca7e1d85f4b2e178a4d7b0b14dda6d5f79b38c571d9911187aa4`.
  The saved file is provenance, not a runtime input or JSBSim dependency.
  Its mass-balance CG is the empty-aircraft reference (41, 0, 36.5) inches.
  Fuel and point masses move the loaded CG; their default loaded CG is not
  used by this fixed-CG comparison.

The XML's aerodynamic reference (43.2, 0, 59.4) inches differs from the PDF's
(43.8, 0, 58.7) inches. The PDF reference is retained with the PDF aerodynamic
coefficients. Using the XML's reference CG with those PDF parameters defines
the stated comparison configuration; this is not a validated reconstruction
of the document's exact aircraft/loading. The original comparison mass and
inertias remain unchanged. AVL coordinates belong to a separate input model.
