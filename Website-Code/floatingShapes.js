function injectFloatingHexagons() {
  if (document.querySelector('.floating-shapes-container')) return;

  const container = document.createElement('div');
  container.className = 'floating-shapes-container';
  container.innerHTML = `
    <svg style="position: absolute; width: 0; height: 0;" aria-hidden="true" focusable="false">
      <defs>
        <linearGradient id="goldGradient" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stop-color="var(--gold, #D4AF37)" />
          <stop offset="100%" stop-color="var(--gold-light, #F3E5AB)" />
        </linearGradient>
      </defs>
    </svg>

    <svg class="float-hex hex-1" viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
      <polygon points="50,5 90,28 90,72 50,95 10,72 10,28" />
    </svg>
    <svg class="float-hex hex-2" viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
      <polygon points="50,5 90,28 90,72 50,95 10,72 10,28" />
    </svg>
    <svg class="float-hex hex-3" viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
      <polygon points="50,5 90,28 90,72 50,95 10,72 10,28" />
    </svg>
  `;
  
  document.body.insertBefore(container, document.body.firstChild);
}
document.addEventListener('DOMContentLoaded', injectFloatingHexagons);