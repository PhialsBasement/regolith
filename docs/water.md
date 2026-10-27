# The water cycle in physical units

mors's GCM carried vapour and three ice "layers" in model units, with
saturation from tuned exponentials and visual coverage remaps in the
renderer. This replaces it with water in kg/m2 on height levels, using
the cloud scheme of Montmessin et al. 2004 (M04 below). Everything else
in the GCM is unchanged.

Sources, each read in full (PDFs from the open archives):

- M04: Montmessin, Forget, Rannou, Cabane, Haberle 2004, "Origin and
  role of water ice clouds in the Martian water cycle as inferred from
  a general circulation model", JGR 109 E10004 (HAL insu-02897064).
- M02: Montmessin, Rannou, Cabane 2002, "New insights into Martian
  dust distribution and water-ice cloud microphysics", JGR 107 E6 5037
  (HAL insu-02897052).
- N14: Navarro et al. 2014, "Global climate modeling of the Martian
  water cycle with improved microphysics and radiatively active water
  ice clouds", JGR Planets 119 (arXiv 1310.1010).
- SK77: Seiff and Kirk 1977, "Structure of the atmosphere of Mars in
  summer at mid-latitudes", JGR 82 (abstract, Tables 3 and 4).
- LMD Mars GCM source, read from svn.lmd.jussieu.fr/Planeto/trunk/
  LMDZ.MARS/libf/phymars: growthrate.F, microphys_h.F90,
  simpleclouds_mod.F, watersat_mod.F90, newsedim_mod.F.

## Units

- Vapour `vap[l]` and ice `ice[l]`, kg/m2 per level; `wv` is the
  vapour column. 1 kg/m2 = 1000 pr-um.
- Surface water ice `frost`, kg/m2. Perennial ice north of 80 N
  (M04 para 34), a 1000 kg/m2 reservoir (M04 gives no amount).

## Vertical grid

Eight levels above the ground, edges 0, 2, 5, 9, 14, 20, 27, 35, 45
km. Pressure p(z) = p_s exp(-z / H), H = R T_a / g. Temperature falls
1.6 K/km to 40 km and is isothermal above (SK77 abstract), centred so
the column mean is the model's T_a; the lowest level is the surface
air. Floored at the CO2 frost point. Orographic lift cools a level by
(g / cp) times the terrain rise across a cell along the wind.

Air molecular weight 43.34 (SK77, Viking 2).

## Saturation

Murphy and Koop 2005 over ice. LMD's watersat_mod.F90 notes it was
checked against the same paper.

## Clouds (M04 sec. 2.2.2)

Crystals form on dust nuclei: N(z) = (tau / 0.1) N0 exp(-z / H), N0
2e6 per m3, tau the column dust opacity; nucleus radius r0(z) = 0.8 um
exp(-z / 18 km) (para 29). Mass mean radius r_c = (M_c / ((4/3) pi
rho_i N) + r0^3)^(1/3) (eq. 2). Growth r dr/dt = (S - S_eq) / (R_c +
R_d) (eq. 3), S_eq the Kelvin term, integrated implicitly so that one
step is stable (para 28, after Jacobson 1997):

M_v' = (M_v + dt k S_eq) / (1 + dt k / M_vs), k = 4 pi N rho_i r_c /
(R_c + R_d), with M_v' at most M_v + M_c.

The resistances are LMD growthrate.F (M02 sec. 2.2): CO2 thermal
conductivity, latent heat, vapour diffusivity with the kinetic
correction. Constants from microphys_h.F90.

## Sedimentation (M04 para 30, 31)

Flux rho_a M_c omega(f r_c) exp(-sigma^2), f = exp(3 sigma^2) = 1.5.
omega is Stokes with the Fuchs slip factor 1 + Kn (1.246 + 0.42
exp(-0.87 / Kn)) (M02 sec. 2.3); viscosity 1e-5 Pa s and mean free
path as LMD newsedim_mod.F. Out of the lowest level onto the ground.

## Vertical mixing

Convective adjustment over the boundary layer (M04 para 32) and eddy
diffusion: 100 m2/s up to 25 km, rising inversely with density above
(M02 para 33, the standard scenario). Both vapour and ice.

## Surface exchange (M04 eq. 1)

E = rho C_d |U| (q_s(T_s) - q), C_d = (kappa / ln(z / z0))^2, kappa
0.4, z0 1 cm, z 4 m. Surface ice over 5 um takes albedo 0.4 (M04 para
35; N14 later tuned 0.35).

## Optical depth (for the renderer)

tau = 3 Q_ext M_c / (4 rho_i r_c), Q_ext 2 in the visible (M04 para 85).

## Start

150 ppm vapour at the north pole falling linearly to 0 at the south
pole, uniform in height (M04 para 37). M04 runs until the inventory
changes by under 1 percent a year; N14 needs about 20 years. Short runs
from this start are a transient.

## Known differences from M04

- M04 uses the model's own temperature profile (25 levels); this GCM
  has one column temperature, so the levels use the SK77 lapse rate.
- Dust is the GCM's own field rather than M04's prescribed scenario.
- No radiative effect of clouds (as M04).

## Targets

- North polar summer vapour peak: 60 pr-um (N14 targets), 80 to 90
  observed (TES via M04 Fig. 2).
- Tropics in northern fall about 10 pr-um (N14).
- Aphelion belt 10 S to 30 N, Ls 40 to 140, 0.15 at 825 cm-1 at 2 pm
  (N14); M04 Fig. 6 gives 0.2 observed and 0.4 modelled at peak.
- Clouds about 9 percent of atmospheric water on the global average,
  following insolation (M04 para 47, Fig. 5).
