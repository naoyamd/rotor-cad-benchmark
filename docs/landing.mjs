// Preserve links shared before the comparison landing page was introduced.
function routeLegacyLink() {
  if (['#explore', '#benchmark', '#evidence'].includes(location.hash)) {
    location.replace(new URL('c1.html' + location.hash, location.href));
  }
}
routeLegacyLink();
addEventListener('hashchange', routeLegacyLink);
