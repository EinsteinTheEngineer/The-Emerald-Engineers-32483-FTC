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
