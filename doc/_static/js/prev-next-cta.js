document.addEventListener("DOMContentLoaded", () => {
  const titles = document.querySelectorAll(".prev-next-area .prev-next-title");
  titles.forEach((title) => {
    const text = title.textContent.replace(/\s+/g, " ").trim();
    if (text && !/\s/.test(text)) {
      title.classList.add("prev-next-title-single");
    }
  });
});
