const $ = (id) => document.getElementById(id);
let products = [];
const fmt = (value) => Number(value || 0).toLocaleString();

async function api(path, options = {}) {
  const headers = new Headers(options.headers || {});
  if (options.body && !headers.has("Content-Type")) headers.set("Content-Type", "application/json");
  const response = await fetch(path, { ...options, headers });
  const body = await response.json().catch(() => ({}));
  if (!response.ok) {
    const detail = body?.detail;
    const text = Array.isArray(detail) ? detail.map((item) => item.msg).join("; ") : detail;
    throw new Error(text || `Request failed (${response.status})`);
  }
  return body;
}
function message(text, kind = "") {
  const node = $("cart-message");
  node.textContent = text;
  node.className = `notice ${kind}`;
}
function readCart() {
  try {
    const items = JSON.parse(sessionStorage.getItem("marsCart") || "[]");
    return new Map(items.filter((item) => item && typeof item.product_id === "string" && Number.isInteger(item.quantity) && item.quantity > 0)
      .map((item) => [item.product_id, item.quantity]));
  } catch (_) { return new Map(); }
}
function saveCart(cart) {
  sessionStorage.setItem("marsCart", JSON.stringify([...cart].map(([product_id, quantity]) => ({ product_id, quantity }))));
  const count = [...cart.values()].reduce((sum, quantity) => sum + quantity, 0);
  $("cart-count").textContent = String(count);
  $("cart-count-side").textContent = `${count} item${count === 1 ? "" : "s"}`;
  $("place-order").disabled = count === 0;
}
function addText(parent, tag, text, className = "") {
  const node = document.createElement(tag);
  node.textContent = text;
  if (className) node.className = className;
  parent.append(node);
  return node;
}
function rememberOrder(order) {
  const key = "marsWarehouseOrders";
  const number = String(order?.order_number || order?.id || "");
  if (!/^\d{6}$/.test(number)) return;
  let orders = [];
  try { orders = JSON.parse(localStorage.getItem(key) || "[]"); } catch (_) {}
  const previous = Array.isArray(orders) ? orders.filter((item) => item?.order_number !== number) : [];
  previous.unshift({
    order_number: number, status: "Order received",
    items: (order.items || []).map((item) => ({ name: item.name, quantity: item.quantity })),
    saved_at: new Date().toISOString(),
  });
  localStorage.setItem(key, JSON.stringify(previous));
}
function renderCart() {
  const lines = $("cart-lines");
  lines.replaceChildren();
  const cart = readCart();
  const availableProducts = new Map(products.map((product) => [product.id, product]));
  for (const id of [...cart.keys()]) if (!availableProducts.has(id)) cart.delete(id);
  if (!cart.size) addText(lines, "p", "Your cart is empty. Add items from the shop.", "muted");
  for (const [id, quantity] of cart) {
    const product = availableProducts.get(id);
    const row = document.createElement("div");
    row.className = "cart-line cart-page-line";
    addText(row, "span", product.name);
    const qty = document.createElement("input");
    qty.type = "number";
    qty.min = "1";
    qty.value = String(quantity);
    qty.setAttribute("aria-label", `Quantity of ${product.name}`);
    qty.addEventListener("change", () => {
      const next = readCart();
      const value = Number(qty.value);
      if (!Number.isInteger(value) || value < 1) next.delete(id);
      else next.set(id, value);
      saveCart(next);
      renderCart();
    });
    const remove = document.createElement("button");
    remove.className = "text-button";
    remove.textContent = "Remove";
    remove.addEventListener("click", () => {
      const next = readCart();
      next.delete(id);
      saveCart(next);
      renderCart();
    });
    row.append(qty, remove);
    lines.append(row);
  }
  saveCart(cart);
}
function friendlyError(error) {
  const text = String(error.message || error);
  if (/insufficient stock|unavailable/i.test(text)) return "That quantity is no longer available. Return to Shop and adjust your cart.";
  if (/mongodb|database transaction/i.test(text)) return "Ordering is temporarily unavailable. Your order was not confirmed; please try again later.";
  if (/failed to fetch|networkerror|network request/i.test(text)) return "We could not reach the order service. Check your connection and try again.";
  return text;
}
async function loadCart() {
  message("Loading your cart…");
  try {
    products = await api("/api/v1/products");
    renderCart();
    message("");
  } catch (error) { message(friendlyError(error), "error"); }
}

$("place-order").addEventListener("click", async () => {
  const cart = readCart();
  if (!cart.size) { message("Add at least one item first.", "error"); return; }
  const byId = new Map(products.map((product) => [product.id, product]));
  for (const [id, quantity] of cart) {
    const available = byId.get(id)?.available_stock ?? 0;
    if (quantity > available) {
      message(`${byId.get(id)?.name || "Item"}: only ${available} currently available. Update the quantity and try again.`, "error");
      return;
    }
  }
  const button = $("place-order");
  button.disabled = true;
  button.textContent = "Submitting…";
  try {
    const payload = JSON.stringify({
      items: [...cart].map(([product_id, quantity]) => ({ product_id, quantity })),
      customer_reference: $("customer-reference").value.trim() || null,
    });
    let pending = null;
    try { pending = JSON.parse(sessionStorage.getItem("marsPendingOrder") || "null"); } catch (_) {}
    if (!pending || pending.payload !== payload) pending = { payload, key: crypto.randomUUID() };
    sessionStorage.setItem("marsPendingOrder", JSON.stringify(pending));
    const order = await api("/store/orders", { method: "POST", headers: { "Idempotency-Key": pending.key }, body: payload });
    rememberOrder(order);
    const result = $("order-result");
    result.replaceChildren();
    addText(result, "strong", "Order confirmed");
    addText(result, "p", `Order ID: ${order.order_number}`);
    addText(result, "p", "Use this six-digit order ID to check your order status.", "fine-print");
    const trackLink = document.createElement("a");
    trackLink.className = "button secondary";
    trackLink.href = `/?order=${encodeURIComponent(order.order_number)}#track`;
    trackLink.textContent = "Track this order";
    result.append(trackLink);
    sessionStorage.removeItem("marsPendingOrder");
    sessionStorage.removeItem("marsCart");
    $("customer-reference").value = "";
    renderCart();
    message("Order placed.", "success");
  } catch (error) {
    message(friendlyError(error), "error");
  } finally {
    button.textContent = "Place order";
    renderCart();
  }
});

loadCart();
