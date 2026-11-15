const $ = (id) => document.getElementById(id);
const state = { products: [] };
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
function message(id, text, kind = "") {
  const node = $(id);
  node.textContent = text;
  node.className = `notice ${kind}`;
}
function showView(name) {
  document.querySelectorAll(".view").forEach((view) => view.classList.toggle("active", view.id === `view-${name}`));
  document.querySelectorAll(".nav-button").forEach((button) => button.classList.toggle("active", button.dataset.view === name));
  if (name === "shop") loadCatalog();
  window.dispatchEvent(new CustomEvent("mars:view", { detail: name }));
  history.replaceState(null, "", `${location.pathname}${location.search}#${name}`);
  if (name === "track") {
    renderYourOrders();
    refreshYourOrders();
    if (!orderListTimer) orderListTimer = setInterval(refreshYourOrders, 5000);
  } else if (orderListTimer) {
    clearInterval(orderListTimer);
    orderListTimer = null;
  }
  window.scrollTo({ top: 0, behavior: "smooth" });
}
function addText(parent, tag, text, className = "") {
  const node = document.createElement(tag);
  node.textContent = text;
  if (className) node.className = className;
  parent.append(node);
  return node;
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
  updateCartCount(cart);
}
function updateCartCount(cart = readCart()) {
  const count = [...cart.values()].reduce((sum, quantity) => sum + quantity, 0);
  $("cart-count").textContent = String(count);
}
function friendlyState(order, task) {
  const orderStates = { queued: "Order received", in_progress: "Robot is preparing your order", delivered: "Delivered", assistance_required: "Our team is checking your order", cancelled: "Cancelled" };
  const taskStates = { queued: "Order received", navigating_to_pick: "Robot is heading to pick", at_pick: "Robot is at pick and preparing your items", scanning: "Robot is checking your items", waiting_for_operator: "Robot is at pick and preparing your items", navigating_to_drop: "Items picked · robot is heading to drop", at_drop: "Out for delivery", delivered: "Delivered", returning_to_dock: "Delivered", completed: "Delivered", assistance_required: "Our team is checking your order" };
  return taskStates[task?.state] || orderStates[order.status] || "Status unavailable";
}
const trackingSteps = ["Order received", "Heading to pick", "Picking items", "Heading to drop", "Out for delivery", "Delivered"];
const SAVED_ORDERS_KEY = "marsWarehouseOrders";
let orderListTimer = null;
let orderRefreshInFlight = false;
const expandedOrderNumbers = new Set();
function readSavedOrders() {
  try {
    const value = JSON.parse(localStorage.getItem(SAVED_ORDERS_KEY) || "[]");
    return Array.isArray(value) ? value.filter((item) => /^\d{6}$/.test(String(item.order_number || ""))) : [];
  } catch (_) { return []; }
}
function rememberOrder(order, redraw = true) {
  const number = String(order?.order_number || order?.id || "");
  if (!/^\d{6}$/.test(number)) return;
  const orders = readSavedOrders();
  const previous = orders.find((item) => item.order_number === number) || {};
  const taskState = order.task?.state || previous.task_state || (order.status === "delivered" ? "delivered" : "queued");
  const saved = {
    order_number: number,
    task_state: taskState,
    order_status: order.status || previous.order_status || "queued",
    status: order.task ? friendlyState(order, order.task) : (previous.status || "Order received"),
    items: (order.items || previous.items || []).map((item) => ({
      name: item.name, quantity: item.quantity, picked: Boolean(item.picked),
    })),
    saved_at: previous.saved_at || new Date().toISOString(),
  };
  const existingIndex = orders.findIndex((item) => item.order_number === number);
  if (existingIndex >= 0 && JSON.stringify(orders[existingIndex]) === JSON.stringify(saved)) return false;
  const next = [...orders];
  if (existingIndex >= 0) next[existingIndex] = saved;
  else next.unshift(saved);
  localStorage.setItem(SAVED_ORDERS_KEY, JSON.stringify(next));
  if (redraw) renderYourOrders();
  return true;
}
function renderYourOrders() {
  const root = $("your-orders-list");
  if (!root) return;
  root.replaceChildren();
  const orders = readSavedOrders();
  $("your-orders-empty").hidden = orders.length > 0;
  for (const order of orders) {
    const details = document.createElement("details");
    details.className = "order-history-item";
    details.open = expandedOrderNumbers.has(order.order_number);
    details.addEventListener("toggle", () => {
      if (details.open) expandedOrderNumbers.add(order.order_number);
      else expandedOrderNumbers.delete(order.order_number);
    });
    const summary = document.createElement("summary");
    summary.className = "order-history-summary";
    const identity = document.createElement("span");
    identity.className = "order-history-identity";
    addText(identity, "strong", `Order ${order.order_number}`);
    const itemNames = (order.items || []).map((item) => `${item.name} × ${item.quantity}`).join(" · ");
    if (itemNames) addText(identity, "small", itemNames);
    addText(summary, "span", order.status || "Order received", "order-history-status");
    summary.prepend(identity);
    details.append(summary);
    const content = document.createElement("div");
    content.className = "order-history-content";
    const current = trackingStepIndex({ task: { state: order.task_state }, status: order.order_status });
    const percent = Math.round((current / (trackingSteps.length - 1)) * 100);
    const progress = document.createElement("div");
    progress.className = "tracking-status-summary compact-progress";
    const orbit = document.createElement("div");
    orbit.className = "tracking-orbit";
    orbit.style.setProperty("--progress", `${percent}%`);
    const core = document.createElement("div");
    core.className = "tracking-orbit-core";
    addText(core, "strong", `${percent}%`);
    addText(core, "span", "journey complete");
    orbit.append(core);
    const statusCopy = document.createElement("div");
    addText(statusCopy, "h3", order.status || "Order received", "tracking-status-title");
    addText(statusCopy, "p", `Order ID ${order.order_number}`, "muted");
    progress.append(orbit, statusCopy);
    content.append(progress);
    const timeline = document.createElement("ol");
    timeline.className = "tracking-timeline";
    trackingSteps.forEach((label, index) => {
      const step = document.createElement("li");
      step.className = `tracking-step${index < current ? " complete" : index === current ? " current" : ""}`;
      if (index === current) step.setAttribute("aria-current", "step");
      addText(step, "span", label);
      timeline.append(step);
    });
    content.append(timeline);
    for (const item of order.items || []) {
      addText(content, "p", `${item.name} · ${item.quantity}${item.picked ? " · picked" : " · being prepared"}`, "order-history-line");
    }
    addText(content, "small", "Status refreshes automatically while the order is active.", "tracking-refresh");
    details.append(content);
    root.append(details);
  }
}
async function refreshYourOrders() {
  if (orderRefreshInFlight) return;
  orderRefreshInFlight = true;
  try {
    let changed = false;
    const active = readSavedOrders().filter((order) =>
      !["delivered", "completed", "cancelled"].includes(order.task_state) &&
      !["delivered", "cancelled"].includes(order.order_status));
    await Promise.all(active.map(async (saved) => {
      try {
        const order = await api(`/store/orders/${encodeURIComponent(saved.order_number)}`);
        changed = rememberOrder(order, false) || changed;
      } catch (_) { /* Keep the saved order visible and retry on the next refresh. */ }
    }));
    if (changed) renderYourOrders();
  } finally {
    orderRefreshInFlight = false;
  }
}
function trackingStepIndex(order) {
  const state = order.task?.state;
  if (state === "navigating_to_pick") return 1;
  if (["at_pick", "scanning", "waiting_for_operator"].includes(state)) return 2;
  if (state === "navigating_to_drop") return 3;
  if (state === "at_drop") return 4;
  if (["delivered", "completed", "returning_to_dock"].includes(state) || order.status === "delivered") return 5;
  return 0;
}
window.addEventListener("storage", (event) => {
  if (event.key === SAVED_ORDERS_KEY && document.getElementById("view-track")?.classList.contains("active")) {
    renderYourOrders();
    refreshYourOrders();
  }
});
function friendlyError(error) {
  const text = String(error.message || error);
  if (/insufficient stock|unavailable/i.test(text)) return "That quantity is no longer available. Refresh the catalogue and update your order.";
  if (/mongodb|database transaction/i.test(text)) return "Ordering is temporarily unavailable. Your order was not confirmed; please try again later.";
  if (/failed to fetch|networkerror|network request/i.test(text)) return "We could not reach the order service. Check your connection and try again.";
  return text;
}
async function loadCatalog() {
  message("catalog-message", "Loading products…");
  try {
    state.products = await api("/api/v1/products");
    renderCatalog();
    message("catalog-message", `${state.products.length} product${state.products.length === 1 ? "" : "s"} available.`);
  } catch (error) { message("catalog-message", friendlyError(error), "error"); }
}
function renderCatalog() {
  const root = $("catalog");
  root.replaceChildren();
  if (!state.products.length) { addText(root, "p", "No products are currently available.", "muted"); return; }
  const cart = readCart();
  for (const product of state.products) {
    const card = document.createElement("article");
    card.className = "product-card";
    addText(card, "h3", product.name);
    const available = Math.max(0, product.available_stock || 0);
    const stock = document.createElement("div");
    stock.className = "stock-line";
    addText(stock, "span", "Available quantity");
    addText(stock, "strong", fmt(available));
    card.append(stock);
    const row = document.createElement("div");
    row.className = "add-row";
    const qty = document.createElement("input");
    qty.type = "number";
    qty.min = "1";
    qty.max = String(available);
    qty.value = available ? "1" : "0";
    qty.disabled = available < 1;
    qty.setAttribute("aria-label", `Quantity of ${product.name}`);
    const add = document.createElement("button");
    add.className = "button primary";
    add.textContent = available ? "Add to cart" : "Out of stock";
    add.disabled = available < 1;
    add.addEventListener("click", () => {
      const count = Number(qty.value);
      if (!Number.isInteger(count) || count < 1 || count > available) return;
      const next = readCart();
      next.set(product.id, (next.get(product.id) || 0) + count);
      saveCart(next);
      add.textContent = "Added";
      setTimeout(() => { add.textContent = "Add to cart"; }, 900);
    });
    row.append(qty, add);
    card.append(row);
    root.append(card);
  }
}

document.querySelectorAll(".nav-button").forEach((button) => button.addEventListener("click", () => showView(button.dataset.view)));
$("reload-catalog").addEventListener("click", loadCatalog);
updateCartCount();
renderYourOrders();
const initialView = location.hash.slice(1);
if (["shop", "track", "admin"].includes(initialView)) showView(initialView);
else loadCatalog();
const requestedOrder = new URLSearchParams(location.search).get("order");
if (requestedOrder && /^\d{6}$/.test(requestedOrder)) {
  expandedOrderNumbers.add(requestedOrder);
  showView("track");
  renderYourOrders();
}
