export function shouldStartInjectionFromKeydown(event, fluoroscopy) {
    return Boolean(
        fluoroscopy &&
        !event?.repeat &&
        (event?.code === 'KeyI' || event?.code === 'KeyC')
    );
}
