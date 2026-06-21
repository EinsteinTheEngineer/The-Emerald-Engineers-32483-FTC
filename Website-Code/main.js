function startCountdown(targetDateStr, elemIdPrefix) {
  const target = new Date(targetDateStr).getTime();
  function update() {
    const now = Date.now();
    const diff = target - now;
    if (diff <= 0) {
      document.getElementById(elemIdPrefix + '-days').textContent = '00';
      document.getElementById(elemIdPrefix + '-hours').textContent = '00';
      document.getElementById(elemIdPrefix + '-minutes').textContent = '00';
      document.getElementById(elemIdPrefix + '-seconds').textContent = '00';
      return;
    }
    const days = Math.floor(diff / (1000*60*60*24));
    const hours = Math.floor((diff % (1000*60*60*24)) / (1000*60*60));
    const minutes = Math.floor((diff % (1000*60*60)) / (1000*60));
    const seconds = Math.floor((diff % (1000*60)) / 1000);
    document.getElementById(elemIdPrefix + '-days').textContent = String(days).padStart(2,'0');
    document.getElementById(elemIdPrefix + '-hours').textContent = String(hours).padStart(2,'0');
    document.getElementById(elemIdPrefix + '-minutes').textContent = String(minutes).padStart(2,'0');
    document.getElementById(elemIdPrefix + '-seconds').textContent = String(seconds).padStart(2,'0');
  }
  update();
  setInterval(update, 1000);
}

document.addEventListener('DOMContentLoaded', () => {
  const menuToggle = document.getElementById('menuToggle');
  const sidebar = document.querySelector('.sidebar');

  // Check to make sure both elements exist on the page
  if (menuToggle && sidebar) {
    menuToggle.addEventListener('click', () => {
      
      // Toggles the .active class on or off every time you click
      sidebar.classList.toggle('active');
      
      // Changes the icon to an '✕' when open, and back to '☰' when closed
      if (sidebar.classList.contains('active')) {
        menuToggle.innerHTML = '✕';
      } else {
        menuToggle.innerHTML = '☰';
      }
    });
  }
});