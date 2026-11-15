(() => {
const $ = (id) => document.getElementById(id);
const state = { products: [], qrUrl: null, selectedIds: new Set() };
const ADMIN_SESSION_KEY = "marsAdminAccessKey";
const ADMIN_SERVER_SESSION_KEY = "marsAdminServerSession";
let signedInAccessKey = sessionStorage.getItem(ADMIN_SESSION_KEY) || "";
let adminServerSessionId = "";
const fmt = (value) => Number(value || 0).toLocaleString();

async function api(path, options = {}, accessKey = "") {
  const headers = new Headers(options.headers || {});
  if (accessKey) headers.set("Authorization", `Bearer ${accessKey}`);
  if (options.body && !headers.has("Content-Type")) headers.set("Content-Type", "application/json");
  const response = await fetch(path, { ...options, headers });
  const type = response.headers.get("content-type") || "";
  const body = type.includes("application/json") ? await response.json() : await response.blob();
  if (!response.ok) throw new Error(body?.detail || `Request failed (${response.status})`);
  return body;
}
function addText(parent, tag, text, className = "") {
  const node = document.createElement(tag);
  node.textContent = text;
  if (className) node.className = className;
  parent.append(node);
  return node;
}
function accessKey() { return signedInAccessKey; }
async function getAdminServerSession() {
  const response = await fetch("/api/v1/session", { cache: "no-store" });
  if (!response.ok) throw new Error("Could not verify the warehouse session.");
  const value = await response.json();
  if (!value.session_id) throw new Error("Warehouse session marker is missing.");
  return value.session_id;
}
async function initializeAdminSession() {
  try {
    adminServerSessionId = await getAdminServerSession();
    if (signedInAccessKey && sessionStorage.getItem(ADMIN_SERVER_SESSION_KEY) !== adminServerSessionId) {
      signedInAccessKey = "";
      sessionStorage.removeItem(ADMIN_SESSION_KEY);
      sessionStorage.removeItem(ADMIN_SERVER_SESSION_KEY);
    }
    if (signedInAccessKey && location.hash === "#admin") loadAdmin();
  } catch (_) {
    signedInAccessKey = "";
    sessionStorage.removeItem(ADMIN_SESSION_KEY);
    sessionStorage.removeItem(ADMIN_SERVER_SESSION_KEY);
  }
}
function message(text, kind = "") { const node = $("admin-message"); node.textContent = text; node.className = `notice ${kind}`; }
function friendlyError(error) {
  if (/invalid bearer|bearer token|insufficient role/i.test(String(error.message))) return "Access was not accepted. Check the admin access key.";
  return error.message || String(error);
}
async function loadAdmin() {
  const key = accessKey();
  if (!key) { message("Enter your admin access key to view inventory."); return; }
  message("Loading inventory…");
  try {
    state.products = await api("/api/v1/admin/products", {}, key);
    state.selectedIds = new Set([...state.selectedIds].filter((id) => state.products.some((p) => p.id === id)));
    renderProducts();
    message(`${state.products.length} items loaded.`);
  } catch (error) {
    if (/invalid bearer|bearer token|insufficient role/i.test(String(error.message))) {
      signedInAccessKey = "";
      sessionStorage.removeItem(ADMIN_SESSION_KEY);
      sessionStorage.removeItem(ADMIN_SERVER_SESSION_KEY);
    }
    message(friendlyError(error), "error");
  }
}
window.addEventListener("mars:view", (event) => {
  if (event.detail === "admin") loadAdmin();
});
function updateSelectionControls() {
  const count = state.selectedIds.size;
  $("print-selected").disabled = count === 0;
  $("print-selected").textContent = count ? `Print ${count} QR tag${count === 1 ? "" : "s"}` : "Print selected QR tags";
  $("qr-selection-status").textContent = count ? `${count} item${count === 1 ? "" : "s"} selected.` : "Select items to print a sheet.";
  $("select-all").textContent = count === state.products.length && count ? "Clear selection" : "Select all";
}
function renderProducts() {
  const root = $("admin-products");
  root.replaceChildren();
  if (!state.products.length) { addText(root, "p", "No items yet.", "muted"); updateSelectionControls(); return; }
  for (const product of state.products) {
    const card = document.createElement("article");
    card.className = "admin-product";
    const head = document.createElement("div");
    head.className = "admin-product-head";
    const heading = document.createElement("div");
    addText(heading, "strong", product.name);
    const selectLabel = document.createElement("label");
    selectLabel.className = "qr-select-label";
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = state.selectedIds.has(product.id);
    checkbox.setAttribute("aria-label", `Select ${product.name} for the QR print sheet`);
    checkbox.addEventListener("change", () => {
      if (checkbox.checked) state.selectedIds.add(product.id);
      else state.selectedIds.delete(product.id);
      updateSelectionControls();
    });
    selectLabel.append(checkbox, document.createTextNode(" Print QR"));
    head.append(heading, selectLabel);
    card.append(head);
    const available = Number(product.stock_on_hand || 0) - Number(product.stock_reserved || 0);
    addText(card, "p", `Quantity ${fmt(product.stock_on_hand)} · Reserved ${fmt(product.stock_reserved)} · Available ${fmt(available)}`, "meta");
    const actions = document.createElement("div");
    actions.className = "admin-actions";
    actions.append(
      smallButton("Rename", "secondary", () => renameProduct(product)),
      smallButton("Adjust quantity", "secondary", () => adjustQuantity(product)),
      smallButton("Preview QR", "secondary", () => showQr(card, product)),
      smallButton("History", "secondary", () => showTransactions(product)),
      smallButton("Delete", "danger", () => deleteProduct(product)),
    );
    card.append(actions);
    root.append(card);
  }
  updateSelectionControls();
}
function smallButton(label, style, callback) {
  const button = document.createElement("button");
  button.className = `button small ${style}`;
  button.textContent = label;
  button.addEventListener("click", callback);
  return button;
}
async function renameProduct(product) {
  const name = prompt("Item name", product.name);
  if (name === null || !name.trim()) return;
  try { await api(`/api/v1/products/${product.id}`, { method: "PATCH", body: JSON.stringify({ name: name.trim() }) }, accessKey()); await loadAdmin(); }
  catch (error) { message(friendlyError(error), "error"); }
}
async function adjustQuantity(product) {
  const deltaText = prompt("Enter the quantity change (positive to add, negative to remove)");
  if (deltaText === null) return;
  const delta = Number(deltaText);
  if (!Number.isInteger(delta) || delta === 0) { message("Enter a nonzero whole number.", "error"); return; }
  const reason = prompt("Reason for this quantity change") || "Manual adjustment";
  try {
    await api(`/api/v1/products/${product.id}/stock-adjustments`, { method: "POST", headers: { "Idempotency-Key": crypto.randomUUID() }, body: JSON.stringify({ delta, reason }) }, accessKey());
    await loadAdmin();
  } catch (error) { message(friendlyError(error), "error"); }
}
async function deleteProduct(product) {
  if (!confirm(`Permanently delete “${product.name}”? This removes the catalogue item even if stock or history exists. Past orders and inventory transactions remain as records.`)) return;
  try {
    await api(`/api/v1/products/${product.id}`, { method: "DELETE" }, accessKey());
    state.selectedIds.delete(product.id);
    message(`${product.name} deleted from the catalogue.`, "success");
    await loadAdmin();
  } catch (error) { message(friendlyError(error), "error"); }
}
async function productQrBlob(product) {
  return api(`/api/v1/products/${product.id}/qr.png`, {}, accessKey());
}
async function showQr(parent, product) {
  try {
    const blob = await productQrBlob(product);
    if (state.qrUrl) URL.revokeObjectURL(state.qrUrl);
    state.qrUrl = URL.createObjectURL(blob);
    parent.querySelector(".qr-card")?.remove();
    const box = document.createElement("div");
    box.className = "qr-card single-qr-card";
    const image = document.createElement("img");
    image.src = state.qrUrl;
    image.alt = `QR tag for ${product.name}`;
    box.append(image);
    addText(box, "strong", product.name, "qr-item-name");
    const download = document.createElement("a");
    download.href = state.qrUrl;
    download.download = `${safeFileName(product.name)}.png`;
    download.className = "button small secondary";
    download.textContent = "Download QR";
    box.append(download);
    parent.append(box);
  } catch (error) { message(friendlyError(error), "error"); }
}
function safeFileName(value) { return String(value).trim().replace(/[^a-z0-9_-]+/gi, "-").replace(/^-|-$/g, "") || "item-qr"; }
function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}
async function printSelectedQrSheet() {
  const selected = state.products.filter((product) => state.selectedIds.has(product.id));
  if (!selected.length) return;
  const printWindow = window.open("", "_blank");
  if (!printWindow) { message("Allow pop-ups to open the QR print sheet.", "error"); return; }
  message("Preparing QR print sheet…");
  let urls = [];
  try {
    const labels = await Promise.all(selected.map(async (product) => {
      const url = URL.createObjectURL(await productQrBlob(product));
      urls.push(url);
      return `<article class="label"><img src="${url}" alt=""><div>${escapeHtml(product.name)}</div></article>`;
    }));
    printWindow.document.open();
    printWindow.document.write(`<!doctype html><html><head><meta charset="utf-8"><title>MARS QR tags</title><style>
      @page{size:A4 portrait;margin:8mm}*{box-sizing:border-box}body{margin:0;font:10pt Arial,sans-serif;color:#111}
      .sheet{display:grid;grid-template-columns:repeat(3,1fr);grid-auto-rows:66mm;gap:3mm;align-content:start}
      .label{border:.25mm dashed #aaa;display:flex;flex-direction:column;align-items:center;justify-content:center;padding:3mm;break-inside:avoid;text-align:center}
      .label img{width:48mm;height:48mm;object-fit:contain}.label div{font-weight:700;font-size:11pt;margin-top:2mm;max-width:100%;overflow-wrap:anywhere}
      @media screen{body{padding:8mm;background:#e9ece8}.sheet{background:white;padding:8mm;width:194mm;min-height:281mm;margin:auto;box-shadow:0 2mm 8mm #0002}}
    </style></head><body><main class="sheet">${labels.join("")}</main></body></html>`);
    printWindow.document.close();
    await Promise.all([...printWindow.document.images].map((image) => image.decode().catch(() => {})));
    printWindow.onafterprint = () => urls.forEach((url) => URL.revokeObjectURL(url));
    printWindow.focus();
    printWindow.print();
    message(`${selected.length} QR tag${selected.length === 1 ? "" : "s"} ready to print.`, "success");
  } catch (error) {
    urls.forEach((url) => URL.revokeObjectURL(url));
    printWindow.close();
    message(friendlyError(error), "error");
  }
}
async function showTransactions(product) {
  const content = $("transactions-content");
  content.replaceChildren();
  addText(content, "p", "Loading…", "muted");
  $("transactions-dialog").showModal();
  try {
    const entries = await api(`/api/v1/products/${product.id}/inventory-transactions`, {}, accessKey());
    content.replaceChildren();
    if (!entries.length) addText(content, "p", "No transactions yet.", "muted");
    for (const entry of entries) {
      const row = document.createElement("div");
      row.className = "transaction-row";
      const left = document.createElement("div");
      addText(left, "strong", entry.type.replaceAll("_", " "));
      addText(left, "small", `${entry.created_at || ""}${entry.reason ? ` · ${entry.reason}` : ""}`);
      addText(row, "span", entry.quantity_delta > 0 ? `+${entry.quantity_delta}` : String(entry.quantity_delta));
      row.prepend(left);
      content.append(row);
    }
  } catch (error) { content.replaceChildren(); addText(content, "p", friendlyError(error), "notice error"); }
}

$("save-admin-token").addEventListener("click", async () => {
  signedInAccessKey = $("admin-token").value.trim();
  $("admin-token").value = "";
  if (!signedInAccessKey) {
    sessionStorage.removeItem(ADMIN_SESSION_KEY);
    sessionStorage.removeItem(ADMIN_SERVER_SESSION_KEY);
    return loadAdmin();
  }
  try {
    adminServerSessionId = await getAdminServerSession();
    sessionStorage.setItem(ADMIN_SESSION_KEY, signedInAccessKey);
    sessionStorage.setItem(ADMIN_SERVER_SESSION_KEY, adminServerSessionId);
  } catch (error) {
    message(error.message, "error");
    return;
  }
  loadAdmin();
});
$("admin-token").addEventListener("keydown", (event) => { if (event.key === "Enter") { event.preventDefault(); $("save-admin-token").click(); } });
$("reload-admin").addEventListener("click", loadAdmin);
$("print-selected").addEventListener("click", printSelectedQrSheet);
$("select-all").addEventListener("click", () => {
  if (state.selectedIds.size === state.products.length) state.selectedIds.clear();
  else state.products.forEach((product) => state.selectedIds.add(product.id));
  renderProducts();
});
$("create-product-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const key = accessKey();
  if (!key) { message("Enter your admin access key first.", "error"); return; }
  const identifier = `MARS-${crypto.randomUUID()}`;
  const body = { name: $("product-name").value.trim(), initial_stock: Number($("product-stock").value), identifier, identifier_type: "qr" };
  const output = $("new-product-result");
  output.replaceChildren();
  addText(output, "p", "Adding item…", "muted");
  try {
    const product = await api("/api/v1/products", { method: "POST", body: JSON.stringify(body) }, key);
    output.replaceChildren();
    addText(output, "p", `${product.name} added, quantity ${fmt(product.stock_on_hand)}.`, "notice success");
    await showQr(output, product);
    event.target.reset();
    $("product-stock").value = "0";
    await loadAdmin();
  } catch (error) { output.replaceChildren(); addText(output, "p", friendlyError(error), "notice error"); }
});
$("close-transactions").addEventListener("click", () => $("transactions-dialog").close());
$("admin-token").value = "";
initializeAdminSession();
})();
