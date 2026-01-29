// Navigation highlighting
document.addEventListener('DOMContentLoaded', function() {
    const currentPath = window.location.pathname;
    const links = document.querySelectorAll('.sidebar-item');
    
    links.forEach(link => {
        const href = link.getAttribute('href');
        if (currentPath.includes(href) || 
            (currentPath.endsWith('/') && href === 'index.html')) {
            link.classList.add('active');
        }
    });
});

// Copy button functionality
document.addEventListener('click', function(e) {
    if (e.target.classList.contains('copy-btn')) {
        const codeBlock = e.target.parentElement;
        const code = codeBlock.textContent.replace('Copy', '').trim();
        
        navigator.clipboard.writeText(code).then(() => {
            const originalText = e.target.textContent;
            e.target.textContent = '✓ Copied!';
            setTimeout(() => {
                e.target.textContent = originalText;
            }, 2000);
        });
    }
});

// Search functionality (placeholder)
document.addEventListener('keydown', function(e) {
    if ((e.metaKey || e.ctrlKey) && e.key === 'k') {
        e.preventDefault();
        const searchBar = document.querySelector('.search-bar');
        if (searchBar) {
            searchBar.focus();
        }
    }
});
