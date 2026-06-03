# cone_fusion

EKF-SLAM a coni che fonde l'odometria di **FAST-LIMO/FAST-LIO** (usata come sorgente di
moto *relativa*) con i **coni statici** della pista (usati come landmark) per ancorare il
drift globale della LIDAR-inertial odometry.

Output principale: una posa odometrica `/Odometry` nel frame `track` che, dal secondo giro
in poi, è riallineata sulla mappa dei coni e quindi non deriva con FAST-LIMO.

---

## 1. Interfaccia ROS

### Subscribe
| Topic | Tipo | Uso |
|---|---|---|
| `/clusters` (`cones_topic`) | `visualization_msgs/Marker` | coni osservati dal LiDAR (cluster), in frame sensore |
| `/fast_limo/state` (`input_odom_topic`) | `nav_msgs/Odometry` | posa + covarianza di FAST-LIMO |
| `/planning/race_status` (`race_status_topic`) | `mmr_base/RaceStatus` | giro corrente (per la logica di "primo giro completato") |

### Publish
| Topic | Tipo | Uso |
|---|---|---|
| `/Odometry` (`output_odom_topic`) | `nav_msgs/Odometry` | posa stimata dall'EKF (frame `track` → `imu_link`) + TF |
| `/slam/cones_positions` (`mapped_cones_topic`) | `visualization_msgs/Marker` | mappa dei coni (viva durante il mapping, congelata dopo) |

### Frame
Tutto lo stato vive nel frame **`track`**, la cui origine coincide con la posa di partenza
del veicolo. La posa è planare: `track → imu_link` con sola traslazione XY e rotazione yaw.

---

## 2. Rappresentazione dello stato

Lo stato è quello classico dell'EKF-SLAM (Thrun, *Probabilistic Robotics*, cap. 10) con
landmark a posizione (range/bearing):

$$
\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \\ m_{1,x} \\ m_{1,y} \\ \vdots \\ m_{N,x} \\ m_{N,y}\end{bmatrix}
\in \mathbb{R}^{2N+3}, \qquad
\mathbf{P} \in \mathbb{R}^{(2N+3)\times(2N+3)}
$$

- $(x,y,\theta)$ = posa 2D del veicolo nel frame `track`.
- $(m_{j,x}, m_{j,y})$ = posizione assoluta del cono $j$.
- $N$ = `N_CONES` = **400** (capacità a compile-time, `ekf_odom.hpp`). La dimensione delle
  matrici è fissa a $2N+3 = 803$; il numero di coni effettivamente mappati è
  `landmark_count` $\le N$.

**Inizializzazione** (`EKFOdom` ctor):

$$
\mathbf{x} = \mathbf{0}, \qquad
\mathbf{P} = \begin{bmatrix} I_3 & 0 \\ 0 & \texttt{INF}\cdot I_{2N}\end{bmatrix},
\quad \texttt{INF}=10
$$

I landmark partono con covarianza "infinita" e cross-covarianza nulla: sono **disaccoppiati**
finché non vengono osservati. Questo è ciò che permette di lavorare solo sul sotto-blocco
attivo (§5).

---

## 3. Passo di predizione (moto relativo da FAST-LIMO)

> **Decisione di design.** FAST-LIMO **non** è trattato come verità assoluta della posa, ma
> come **odometria relativa**. Lo stato EKF è ancorato all'origine `track`; ad ogni messaggio
> si applica solo l'*incremento* di moto di FAST-LIMO. Così la correzione dei coni rimane
> nello stato invece di essere sovrascritta dalla posa grezza ad ogni frame.

`setPose(p_k)` con $p_k = (x_k^{L}, y_k^{L}, \theta_k^{L})$ posa di FAST-LIMO:

**Primo frame** ($\texttt{is\_pose\_initialized}=\text{false}$): si registra solo il
riferimento, lo stato resta all'origine.

$$
p_{\text{prev}} \leftarrow p_k, \qquad \mathbf{x}_{0:3} \;\text{invariato} \;(=\mathbf{0})
$$

**Frame successivi:** si calcola lo spostamento relativo nel body-frame del frame precedente
e lo si compone sulla posa EKF (composizione rigida $T_{\text{ekf}} \leftarrow
T_{\text{ekf}}\cdot T_{\text{prev}}^{-1}\cdot T_k$):

$$
\begin{aligned}
\Delta_x &= x_k^L - x_{\text{prev}}^L, \quad \Delta_y = y_k^L - y_{\text{prev}}^L \\
\ell_x &= \;\;\,\cos\theta_{\text{prev}}\,\Delta_x + \sin\theta_{\text{prev}}\,\Delta_y \\
\ell_y &= -\sin\theta_{\text{prev}}\,\Delta_x + \cos\theta_{\text{prev}}\,\Delta_y \\
\Delta_\theta &= \operatorname{wrap}(\theta_k^L - \theta_{\text{prev}}^L)
\end{aligned}
$$

$$
\begin{aligned}
x &\mathrel{+}= \cos\theta\,\ell_x - \sin\theta\,\ell_y \\
y &\mathrel{+}= \sin\theta\,\ell_x + \cos\theta\,\ell_y \\
\theta &\leftarrow \operatorname{wrap}(\theta + \Delta_\theta)
\end{aligned}
$$

**Covarianza — propagazione** (in `setPose`): la posa-incremento è anche propagata attraverso
il Jacobiano del moto $G$, esattamente come il passo *predict* di un EKF:

$$
\mathbf{P} \leftarrow G\,\mathbf{P}\,G^\top, \qquad
G = \begin{bmatrix} 1 & 0 & -d_{w,y} \\ 0 & 1 & \;\;d_{w,x} \\ 0 & 0 & 1\end{bmatrix} \;\text{(blocco posa; } I \text{ altrove)}
$$

dove $(d_{w,x}, d_{w,y}) = R(\theta)\,(\ell_x,\ell_y)$ è lo spostamento in frame mondo. $G$
accoppia l'incertezza di heading nella covarianza di posizione e **ruota le cross-covarianze
posa–landmark**; senza, $\mathbf{P}$ crescerebbe solo sulla diagonale e le correzioni si
distribuirebbero male tra $x,y,\theta$. Si opera solo sul sotto-blocco attivo (i landmark
inattivi sono disaccoppiati), quindi il costo è $O(n_a)$.

**Covarianza — rumore di processo** (`setPoseCovariance`): dopo la propagazione si somma
$Q_{\text{motion}}$, cioè **solo l'incremento** di covarianza di FAST-LIMO (non il valore
assoluto, che sommato ad ogni frame ad alto rate farebbe esplodere $\mathbf{P}$):

$$
P_{ii} \mathrel{+}= \max\!\big(0,\; \sigma^{L}_{k,i} - \sigma^{L}_{k-1,i}\big), \quad i\in\{0,1,2\}
$$

Il clamp a $\ge 0$ evita che un eventuale calo della covarianza di FAST-LIMO (es. una sua
relocalizzazione) restringa erroneamente $\mathbf{P}$: il restringimento deve venire **solo**
dai coni, in `correct()`. Insieme, i due passi realizzano il classico
$\mathbf{P}\leftarrow G\mathbf{P}G^\top + Q_{\text{motion}}$.

> **Nota.** Esiste anche un `predict(dt)` con modello cinematico a velocità/velocità angolare
> ($v$, $\omega$), ma **non è collegato** nella pipeline attuale (`setActVel`/`setActAngVel`
> non vengono mai chiamati). La predizione è interamente fatta dall'incremento relativo qui
> sopra. Il metodo è lasciato per un eventuale uso futuro con un'IMU/encoder.

---

## 4. Passo di correzione (coni)

`correct(z, M)` riceve $M$ osservazioni $z_i = (\rho_i, \phi_i, c_i)$ = range, bearing,
colore, calcolate in `conesCallback` dai punti del cluster:

$$
\rho_i = \sqrt{p_x^2 + p_y^2}, \qquad \phi_i = \operatorname{wrap}\!\big(\operatorname{atan2}(p_y, p_x)\big)
$$

### 4.1 Associazione dati (nearest-neighbour)

Per ogni osservazione si proietta il cono nel frame globale data la posa corrente:

$$
\hat{m}_i = \begin{bmatrix} x \\ y \end{bmatrix} + \rho_i \begin{bmatrix}\cos(\phi_i + \theta)\\ \sin(\phi_i + \theta)\end{bmatrix}
$$

e si cerca il landmark mappato più vicino in distanza euclidea, indice $k$, distanza
$d_{\min}$.

- Se $d_{\min} > \alpha$ (`min_new_cone_distance`):
  - **solo durante il 1° giro** → nuovo landmark: $m_k \leftarrow \hat{m}_i$, `landmark_count++`.
  - **dal 2° giro** → osservazione scartata (`continue`): la mappa non cresce più (ancora fissa).
- Altrimenti → associazione al landmark $k$ (e aggiornamento del contatore colore).

I coni **arancioni** ($c\in\{2,3\}$) sono scartati.

### 4.2 Modello di misura e Jacobiano

Per il landmark associato $k$, con $\delta = m_k - (x,y)^\top$ e $q = \delta^\top\delta$:

$$
\hat{z} = h(\mathbf{x}) = \begin{bmatrix}\sqrt{q}\\ \operatorname{wrap}\big(\operatorname{atan2}(\delta_y,\delta_x) - \theta\big)\end{bmatrix}
$$

Jacobiano "basso" $2\times 5$ rispetto a $[x,y,\theta,m_{k,x},m_{k,y}]$:

$$
{}^{low}H = \frac{1}{q}\begin{bmatrix}
-\sqrt{q}\,\delta_x & -\sqrt{q}\,\delta_y & 0 & \sqrt{q}\,\delta_x & \sqrt{q}\,\delta_y \\
\delta_y & -\delta_x & -q & -\delta_y & \delta_x
\end{bmatrix}
$$

mappato sullo stato tramite $F_{x,k}$ (seleziona le 3 colonne della posa e le 2 del landmark
$k$): $H = {}^{low}H \, F_{x,k}$.

### 4.3 Aggiornamento di Kalman

$$
S = H\,\mathbf{P}\,H^\top + Q \;(2\times2), \qquad
K = \mathbf{P}\,H^\top S^{-1}, \qquad
\mathbf{x} \mathrel{+}= K\,(z - \hat z), \qquad
\mathbf{P} \leftarrow \mathbf{P} - K\,(H\mathbf{P})
$$

dove $Q$ = `proc_noise` (vedi §6, nomenclatura non standard). L'update di $\mathbf{P}$ è scritto
come $\mathbf{P} - K(H\mathbf{P})$, algebricamente identico a $(I-KH)\mathbf{P}$ ma in $O(n_a^2)$
invece di $O(n^3)$ (§5).

**Wrapping dell'innovazione.** La componente di bearing dell'innovazione $\nu = z - \hat z$
viene normalizzata a $[-\pi,\pi]$: sia $\phi_i$ sia $\hat z_\theta$ sono già normalizzati
singolarmente, ma la loro **differenza** può cadere a cavallo di $\pm\pi$ (es. $+3.0-(-3.0)=6.0$
rad invece di $-0.28$). Senza il wrapping, un cono ri-osservato attraverso la discontinuità
angolare — tipico alla chiusura del giro — inietterebbe un'innovazione enorme e spuria che fa
"scattare" la posa.

### 4.4 Gating di associazione (Mahalanobis)

Prima di applicare l'update, l'innovazione è testata con la distanza di Mahalanobis normalizzata,
che segue una chi-quadro a 2 gradi di libertà (range + bearing):

$$
d^2 = \nu^\top S^{-1} \nu, \qquad \nu = z - \hat z
$$

Se $d^2 > 9.21$ (bound al 99% per 2 DoF) l'osservazione è grossolanamente incoerente con il
landmark associato — quasi sempre un'**associazione errata** (es. alla chiusura del giro, quando
si ri-osservano i primi coni con il drift accumulato) — e l'update viene **scartato**
(`continue`). Il gate è attivo **solo dal 2° giro** (`is_first_lap_completed`): durante il 1°
giro la posa è comunque congelata ($K_{0:3}=0$) e la mappa si sta ancora formando, quindi tutte
le associazioni vengono lasciate raffinare i landmark. Poiché il gate può solo **scartare**
update, non restringe mai $\mathbf{P}$ in modo improprio.

### 4.5 Reiezione dei falsi positivi (rapporto rilevazioni/visibilità)

Un contatore cumulativo di "quante volte un cono è stato visto" non distingue **5 rilevazioni
in un giro** (cono reale) da **5 rilevazioni sparse su 6 giri** (falso positivo): cresce in modo
monotono e, su molti giri, anche i ghost finiscono per superare la soglia. Per questo ogni
landmark mantiene **due** contatori (`color_logic.hpp`):

- `detected` — incrementato ad ogni associazione (in `setColor`, §4.1);
- `expected` — incrementato ad ogni scan in cui il landmark cade dentro il FOV/range del sensore.

`expected` è aggiornato da `updateLandmarkVisibility(max_range, half_fov)`, chiamato una volta
per scan in `conesCallback` dopo `correct()`: per ogni cono mappato si predice range/bearing
dalla posa corrente e, se $\rho < \texttt{lidar\_max\_range}$ e $|\phi| < \texttt{lidar\_fov}/2$,
si fa `expected++`.

Un cono è pubblicato solo se supera **entrambe** le soglie:

$$
\texttt{detected} \ge \texttt{cone\_time\_seen\_th}
\quad\wedge\quad
\frac{\texttt{detected}}{\texttt{expected}} \ge \texttt{cone\_confidence\_th}
$$

Il rapporto $\approx 1$ per un cono reale (rilevato quasi ad ogni passaggio in cui è visibile)
e resta basso per un ghost che combacia solo sporadicamente — es. visto 5 volte ma in FOV 50+
volte su 6 giri $\Rightarrow 0.1$, scartato. Con `cones_pub_for_debug: true` il gate è
rivalutato ad ogni scan su tutti i giri, quindi il rapporto si stringe man mano che si accumula
evidenza.

---

## 5. Decisioni di design e ottimizzazioni

### Ancora attiva solo dal 2° giro
La forza dell'ancora dipende da una mappa matura. Per evitare correzioni rumorose all'avvio
(mappa sparsa, geometria povera, associazioni incerte), durante il **1° giro** la correzione
**non muove la posa**: si azzerano le righe-posa del guadagno,

$$K_{0:3} \leftarrow 0 \quad \text{se } \neg\,\texttt{is\_first\_lap\_completed}$$

così il 1° giro **costruisce e raffina la mappa** mentre la posa segue FAST-LIMO in puro
relativo. `is_first_lap_completed` diventa `true` quando `current_lap > 1`. Dal 2° giro il
guadagno è pieno → la posa viene ancorata, e non si aggiungono più coni.

### Sotto-blocco attivo + update $O(n_a^2)$
Le matrici sono dimensionate a $n=803$ ma i landmark non mappati sono `INF·I` disaccoppiati:
le loro righe di $K$ sono comunque nulle. Tutte le operazioni di `correct()` girano quindi
sul solo blocco attivo $n_a = 3 + 2\cdot\texttt{landmark\_count}$. Inoltre l'update della
covarianza usa la forma $\mathbf{P}-K(H\mathbf{P})$ ($O(n_a^2)$) invece del prodotto pieno
$(I-KH)\mathbf{P}$ ($O(n^3)$). Questo è essenziale su Jetson Orin (IPC CPU modesto), e scala
bene se si aumenta `N_CONES`.

### Una sola sorgente di pubblicazione
- `/Odometry` è pubblicata **solo** da `fastLimoDataCallback` (rate FAST-LIMO, alto), leggendo
  lo stato EKF `getState().head(3)` — **non** la posa LIMO grezza.
- I marker dei coni sono pubblicati **solo** da `conesCallback`, per evitare che due set diversi
  (mappa viva vs congelata) finiscano sullo stesso topic facendo "vibrare" i coni in RViz.

---

## 6. Tuning dei parametri (`config/config.yaml`)

> ⚠️ **Nomenclatura non standard.** Nel codice i due rumori sono mappati al contrario rispetto
> alla convenzione EKF, e sono usati come **varianza** (il quadrato è commentato, quindi il
> valore inserito è $\sigma^2$, non $\sigma$, nonostante i commenti dicano "Sigma"):
> - `noises.proc_noise` → matrice $Q$ (2×2), **covarianza di innovazione della misura** in `correct()`.
> - `noises.meas_noise` → matrice $R$ (3×3), rumore di processo di `predict()` → **attualmente inerte** (predict non è chiamato).

| Parametro | Effetto | Come tararlo |
|---|---|---|
| `noises.proc_noise` `[var_range, var_bearing]` | Quanto il filtro si fida dei coni. È la $Q$ in $S=HPH^\top+Q$. **Grande** → coni meno credibili → correzioni dolci, ancoraggio lento, meno jitter. **Piccolo** → correzioni aggressive, ancoraggio rapido, più jitter e rischio di divergenza con associazioni errate. | Parti da `[0.1, 0.1]`. Se i coni "vibrano"/la posa scatta dal 2° giro, **aumenta**. Se l'ancora è troppo lenta a recuperare il drift, **diminuisci**. `var_bearing` in rad²: 0.1 ≈ σ≈18°, piuttosto largo. |
| `noises.meas_noise` `[x,y,yaw]` | $R$ di `predict()`. **Inerte** finché `predict()` non viene collegato. | Lasciare com'è; rilevante solo se si attiva il modello a velocità. |
| `noises.min_new_cone_distance` ($\alpha$) [m] | Soglia di associazione / creazione nuovo cono. **Grande** → meno coni nuovi, associazione più aggressiva (rischio di fondere coni distinti). **Piccolo** → più coni (rischio duplicati da rumore). | Impostare **sotto la metà** della spaziatura minima tra coni adiacenti in pista e **sopra** il rumore di posizione per-frame. Default `2.0`. |
| `generic.cone_time_seen_th` | Soglia **assoluta minima** di rilevazioni perché un cono sia eleggibile alla pubblicazione (combinata con il rapporto `cone_confidence_th`, §4.5). **Alto** → mappa più pulita ma coni che compaiono tardi. | `4` è un buon compromesso; alza se vedi coni spuri, abbassa se la mappa si popola troppo lentamente. |
| `generic.lidar_max_range` [m] | Range entro cui un cono è considerato "atteso" (`expected++`) nel rapporto di confidenza (§4.5). **Troppo grande** → coni reali oltre la portata reale del percettore accumulano `expected` senza `detected` e vengono scartati per errore. | Impostare al range entro cui il **percettore** rileva i coni in modo affidabile, **non** al range massimo grezzo del LiDAR. |
| `generic.lidar_fov` [deg] | FOV orizzontale (angolo pieno) usato per il test di visibilità (§4.5). | Impostare al FOV effettivo del percettore di coni. |
| `generic.cone_confidence_th` `[0..1]` | Soglia minima del rapporto `detected/expected` per pubblicare un cono (reiezione FP, §4.5). **Alto** → mappa più pulita ma rischio di scartare coni reali ai bordi; **basso** → più permissivo. | Parti da `0.3`; alza verso `0.5–0.6` se i ghost passano ancora. Se vengono scartati coni reali ai bordi, **prima** restringi `lidar_max_range`/`lidar_fov`. |
| `generic.cones_pub_for_debug` | `true` → pubblica la mappa **viva** dell'EKF anche dopo il 1° giro (debug). `false` → pubblica la mappa **congelata**. | Tieni `false` in gara. In debug ricorda che la viva si muove (i coni associati sono ancora corretti dal filtro). |
| `generic.is_colorblind` | `true` → tutti i coni trattati come gialli (colore ignorato nell'associazione). | Lasciare `true` se il colore dal percettore non è affidabile. |
| `generic.is_skidpad_mission` | Modalità skidpad: pubblica solo posa, niente marker coni. | `false` per missioni con mappatura coni. |
| `N_CONES` (compile-time, `ekf_odom.hpp`) | Capacità massima di landmark. Aumentarlo ingrandisce le matrici (costo $\propto N$ sul blocco inattivo, ma `correct()` lavora solo su $n_a$). | Alzare se la pista ha più di ~400 coni. |

---

## 7. Assunzioni

1. **Moto planare**: si stimano solo $(x,y,\theta)$; $z$, roll, pitch sono ignorati. Lo yaw è
   estratto dal quaternione di FAST-LIMO.
2. **FAST-LIMO localmente accurato**: la deriva è piccola sul breve periodo (un giro) e si
   accumula sul lungo periodo → i coni la ancorano dal 2° giro.
3. **Mondo statico**: i coni non si muovono; sono un riferimento globale fisso.
4. **Partenza all'origine**: il veicolo parte nell'origine di `track` ($\mathbf{x}=\mathbf{0}$);
   di FAST-LIMO si usa **solo il moto relativo**, quindi il suo frame assoluto è irrilevante.
5. **Osservazioni in range/bearing** da cluster LiDAR, riferite all'origine del veicolo (nessun
   offset estrinseco sensore→veicolo applicato).
6. **Una sola correzione di misura per frame coni**: attualmente l'update di Kalman viene
   eseguito solo sull'**ultimo** cono della lista (`i == M-1`); gli altri sono solo
   associati/mappati. Vedi §8.
7. **Coni arancioni scartati** nell'aggiornamento.

---

## 8. Limitazioni note / TODO

- **Update a singolo cono per frame** (`i == M-1`): sotto-vincola la posa (una misura range/
  bearing fissa ~2 DoF su 3), dipende dall'ordine di detection e rende le correzioni "nervose".
  Fondere tutte le associazioni del frame (update sequenziale o batch) ridurrebbe la varianza
  di ~$1/M$. Con l'update già in $O(n_a^2)$ il costo è sostenibile anche su Orin.
- **Coni non congelati dal 2° giro**: le righe-landmark di $K$ restano non nulle, quindi i coni
  associati sono ancora leggermente mossi. Per un'ancora *rigida* si possono azzerare anche
  quelle righe quando `is_first_lap_completed` (simmetrico al gating del 1° giro sulla posa).
- **`predict()` scollegato**: il modello a velocità è codice dormiente.
- **Membro `Fx_k` morto**: sostituito da `Fx_k_a` locale in `correct()`; rimuovibile.
- **Nomenclatura rumori invertita** e usata come varianza: vedi §6.

---

## 9. Flusso dati (riassunto)

```
/fast_limo/state ──▶ fastLimoDataCallback
                      ├─ setPose()            : x ⊕= incremento relativo LIMO   (predict)
                      ├─ setPoseCovariance()  : P[pose] += ΔcovLIMO  (clamp ≥0)
                      └─ updatePose()         : pubblica /Odometry da getState() (alto rate)

/clusters ──────────▶ conesCallback
                      ├─ correct()                 : associazione NN + wrapping innovazione +
                      │                               gate Mahalanobis + update Kalman (blocco attivo)
                      │                               (1° giro: K[pose]=0 → solo mapping;
                      │                                2° giro: K pieno → ancora la posa, no nuovi coni)
                      ├─ updateLandmarkVisibility() : expected++ per i coni mappati dentro FOV/range
                      └─ pubConesMarkers()         : pubblica i coni con detected ≥ cone_time_seen_th
                                                      e detected/expected ≥ cone_confidence_th

/planning/race_status ─▶ raceStatusCallback   : current_lap → setFirstLapCompleted(lap>1)
```
