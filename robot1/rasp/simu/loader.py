import json
import sys
from pathlib import Path
from typing import Any, Dict
import importlib

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

# Instance globale du loader
loader = ModuleLoader()