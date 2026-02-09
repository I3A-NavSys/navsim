import omni

stage = omni.usd.get_context().get_stage()

vertiport_prims = []

# Obtenemos todos los Prims del stage y filtramos aquellos que sean de tipo "vertiport"
for prim in stage.TraverseAll():
    if prim.GetAttribute("NavSim:type").Get() == "vertiport":
        vertiport_prims.append(prim)

for vertiport in vertiport_prims:
    id = vertiport.GetAttribute("NavSim:id").Get()
    model = vertiport.GetAttribute("NavSim:model").Get()
    type = vertiport.GetAttribute("NavSim:type").Get()
    position = vertiport.GetAttribute("xFormOp:translate").Get()

    print(f"Vertiport ID: {id}, Model: {model}, Type: {type}, Position: {position}")

def find_and_parse_custom_prims():
    """
    Recorre el stage actual buscando Prims que tengan una etiqueta específica
    y extrae información adicional de ellos.
    """
    # 1. Obtener el contexto y el stage actual (API estándar de Omniverse)
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()

    if not stage:
        print("[ERROR] No hay un stage abierto en Isaac Sim.")
        return

    # --- CONFIGURACIÓN DE TUS ETIQUETAS ---
    # Nombre del atributo que actúa como "bandera" o etiqueta de identificación
    TAG_IDENTIFIER = "navsim:generated" 
    # Nombre del atributo que contiene la información que quieres leer
    TAG_INFO = "user:description" 
    # --------------------------------------

    found_objects = []

    print(f"[INFO] Iniciando búsqueda de Prims con atributo: '{TAG_IDENTIFIER}'...")

    # 2. Recorrer la escena (Traverse)
    # Traverse() es un iterador eficiente que recorre toda la jerarquía
    for prim in stage.Traverse():
        
        # 3. Comprobar si tiene el atributo identificador
        if prim.HasAttribute(TAG_IDENTIFIER):
            attr_id = prim.GetAttribute(TAG_IDENTIFIER)
            
            # (Opcional) Verificar que el valor sea True si es un booleano
            # Si tu etiqueta es solo "existencia", puedes borrar el "if is_generated:"
            is_generated = attr_id.Get()
            
            if is_generated:
                # 4. Extraer la información del otro atributo
                prim_data = {
                    "path": str(prim.GetPath()),
                    "name": prim.GetName(),
                    "info_value": "N/A" # Valor por defecto
                }

                # Intentar leer la etiqueta de información
                if prim.HasAttribute(TAG_INFO):
                    info_attr = prim.GetAttribute(TAG_INFO)
                    prim_data["info_value"] = info_attr.Get()
                else:
                    print(f"[WARN] El objeto {prim.GetName()} tiene la marca, pero no el campo de info '{TAG_INFO}'")

                found_objects.append(prim_data)

    # 5. Mostrar resultados
    print("-" * 60)
    print(f"[RESULTADO] Se encontraron {len(found_objects)} objetos generados por el usuario.")
    print("-" * 60)
    
    for obj in found_objects:
        print(f" > Prim: {obj['name']}")
        print(f"   Ruta: {obj['path']}")
        print(f"   Info: {obj['info_value']}")
        print("-" * 20)

    return found_objects

# Ejecutar la función
objetos_encontrados = find_and_parse_custom_prims()
