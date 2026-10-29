const escape = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
export const bi = (en, ja) => `<span class="en" lang="en">${escape(en)}</span><span class="ja" lang="ja">${escape(ja)}</span>`;
