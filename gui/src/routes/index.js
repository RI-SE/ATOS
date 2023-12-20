var express = require('express');
var router = express.Router();

/* GET home page. */
router.get('/controlpanel', function(req, res, next) {
  res.render('controlpanel', { title: 'Simple Control GUI for ATOS' });
});
router.get('/configpanel', function(req, res, next) {
  res.render('configpanel', { title: 'Simple Config GUI for ATOS' });
});


module.exports = router;
