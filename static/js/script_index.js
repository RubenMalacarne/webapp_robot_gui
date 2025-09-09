function loadPage(page, tab) {
  document.getElementById('content-frame').src = page;
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  tab.classList.add('active');
}
