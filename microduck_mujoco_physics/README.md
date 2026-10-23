# Microduck — MuJoCo Physical Lab (base d'intégration)

Ce projet **remplace le moteur cinématique** par MuJoCo WebAssembly et le MJCF officiel du Microduck.

## État et limites

- Le XML MJCF officiel et les STL sont récupérés par `npm run assets`.
- MuJoCo calcule la gravité, les contacts et les positions physiques. Les géométries sont synchronisées avec Three.js via `geom_xpos` et `geom_xmat`.
- La commande expérimentale « marcher » **n'est pas** la politique ONNX officielle : elle ne garantit ni équilibre ni locomotion. Les corps peuvent tomber.
- L'interface FlyWire, les ONNX et la cognition autonome ne sont PAS intégrés.
- **Pas encore validé en navigateur** : l'environnement de génération ne pouvait pas télécharger les dépendances et assets. Il faudra tester et corriger la compatibilité du MJCF et les API wasm selon la version MuJoCo.

## Lancer et tester

Il faut Node.js 20+, Python 3 et une connexion initiale :

```sh
npm install
npm run assets
npm run dev
```

Ouvrir l'adresse locale Vite indiquée. Pour créer un dossier statique Netlify Drop :

```sh
npm run build
```

Déposer le dossier `dist` sur Netlify Drop. Cette version **source** n'est pas directement déposable sur Netlify Drop : il faut d'abord la compiler.

## Étape suivante pour une locomotion crédible

Intégrer les politiques ONNX du simulateur officiel de Pollen Robotics, et traduire leurs observations/actions vers les joints et actionneurs exacts du modèle. Tester l'équilibre et les contacts dans MuJoCo avant d'activer le contrôleur autonome de haut niveau.

Sources :
- https://huggingface.co/spaces/pollen-robotics/microduck-simulator/tree/main
- https://github.com/pollen-robotics/microduck_rl
- https://github.com/google-deepmind/mujoco/tree/main/wasm

Les maillages officiels possèdent leur propre licence. Conserver les attributions et respecter les conditions d'utilisation.
