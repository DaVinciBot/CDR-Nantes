import json
import sys
import logging
from pathlib import Path
from typing import Any, Dict
import importlib
import importlib.util

class ModuleLoader:
    """Charge les modules à partir d'un fichier de configuration JSON"""
    
    def __init__(self, config_file: str = "config.json"):
        self.config_file = Path(__file__).parent / config_file
        self.config = self._load_config()
        self._add_paths_to_sys()
    
    def _load_config(self) -> Dict:
        """Charge le fichier de configuration JSON"""
        with open(self.config_file, 'r') as f:
            return json.load(f)
    
    def _add_paths_to_sys(self):
        """Ajoute les chemins des modules au sys.path"""
        for module_config in self.config.values():
            if isinstance(module_config, dict) and 'module_path' in module_config:
                module_path = Path(__file__).parent / module_config['module_path']
                module_path = module_path.resolve()
                if str(module_path) not in sys.path:
                    sys.path.insert(0, str(module_path))
                
                # Ajouter aussi le parent pour permettre les imports absolus
                parent_path = module_path.parent
                if str(parent_path) not in sys.path:
                    sys.path.insert(0, str(parent_path))
    
    def load_class(self, category: str, class_name: str) -> Any:
        """
        Charge une classe depuis la configuration
        
        Args:
            category: Catégorie dans le JSON (ex: 'usb_com', 'teensy')
            class_name: Nom de la classe à charger (ex: 'Com', 'Messages')
        
        Returns:
            La classe chargée
        """
        if category not in self.config:
            raise ValueError(f"Catégorie '{category}' introuvable dans la configuration")
        
        classes = self.config[category].get('classes', {})
        if class_name not in classes:
            raise ValueError(f"Classe '{class_name}' introuvable dans la catégorie '{category}'")
        
        module_path = classes[class_name]
        module_name, class_name_in_module = module_path.rsplit('.', 1)
        
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError:
            # Essayer avec __import__ comme fallback
            module = __import__(module_name, fromlist=[class_name_in_module])
        
        return getattr(module, class_name_in_module)
    
    def get_config(self, key: str) -> Any:
        """Récupère une valeur de configuration"""
        return self.config.get(key)
    
    def load_rerun_bridge(self):
        """
        Charge le module rerun_bridge depuis le dossier rerun/.
        Expose le module dans sys.modules pour qu'il soit accessible partout.
        
        Returns:
            Le module rerun_bridge, ou None si indisponible
        """
        required_functions = (
            "update_fused",
            "update_obstacles",
            "update_target",
            "update_trajectory",
        )

        try:
            rerun_path = Path(__file__).parent / "rerun" / "rerun_bridge.py"

            if not rerun_path.exists():
                return None

            # Réutiliser uniquement si le module déjà présent est valide
            existing = sys.modules.get("rerun_bridge")
            if existing is not None and all(
                callable(getattr(existing, fn, None)) for fn in required_functions
            ):
                return existing

            # Nettoyer toute version incomplète/partielle
            sys.modules.pop("rerun_bridge", None)

            # Charger le module local sans l'exposer tant qu'il n'est pas valide
            spec = importlib.util.spec_from_file_location("rerun_bridge_impl", rerun_path)
            if spec is None or spec.loader is None:
                return None

            rerun_bridge_module = importlib.util.module_from_spec(spec)
            # Important: certains décorateurs (ex: dataclass) attendent que le
            # module soit présent dans sys.modules pendant l'exécution.
            sys.modules[spec.name] = rerun_bridge_module
            spec.loader.exec_module(rerun_bridge_module)

            if not all(
                callable(getattr(rerun_bridge_module, fn, None)) for fn in required_functions
            ):
                logging.warning("rerun_bridge chargé mais API incomplète")
                return None

            # Exposer uniquement une fois le module validé
            sys.modules["rerun_bridge"] = rerun_bridge_module
            return rerun_bridge_module
        except Exception as e:
            # Éviter qu'un module partiellement chargé reste visible
            sys.modules.pop("rerun_bridge", None)
            sys.modules.pop("rerun_bridge_impl", None)
            logging.warning(f"Impossible de charger rerun_bridge: {e}")
            return None

# Instance globale du loader
loader = ModuleLoader()